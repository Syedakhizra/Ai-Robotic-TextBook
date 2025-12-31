import os
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
import uuid
import re # Import regex module
import cohere # Import the cohere library
from urllib.parse import urljoin, urlparse
import glob # For local file system crawling
import logging # Import the logging module
from fastapi import FastAPI, HTTPException # Import FastAPI
import typer # For CLI commands
from typing import List
from models import QueryRequest, RetrievalResult, ChatRequest, ChatResponse # Added ChatRequest, ChatResponse
from fastapi.middleware.cors import CORSMiddleware # Import CORSMiddleware

# Import components from agent.py (now in the same directory)
from agent import Agent, Runner, function_tool, retrieve_from_qdrant # Corrected import

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Initialize FastAPI app
app = FastAPI()

# Add CORS middleware
origins = [
    "http://localhost",
    "http://localhost:3000", # Assuming your frontend might run on port 3000
    # You can add other origins here for your deployed frontend
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Load environment variables (globally for all modules)
load_dotenv()

# --- Constants ---
MAX_CHUNK_SIZE_CHARS = 1000
OVERLAP_CHARS = 100
EMBEDDING_MODEL = "embed-english-light-v3.0"
EMBEDDING_DIMENSION = 384
QDRANT_COLLECTION_NAME = "reg_embedding"

# --- Helper Functions ---

# These getters now rely on the globally loaded .env variables
def get_qdrant_client() -> QdrantClient:
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    if not QDRANT_URL or not QDRANT_API_KEY:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY not found in .env file.")
    return QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

def get_cohere_client() -> cohere.Client:
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY not found in .env file.")
    return cohere.Client(COHERE_API_KEY)


def get_all_urls(sitemap_url: str = None, local_docs_path: str = None) -> list[str]:
    urls = []
    if sitemap_url:
        try:
            response = requests.get(sitemap_url)
            response.raise_for_status()
            soup = BeautifulSoup(response.content, 'lxml') # Use lxml
            
            for loc in soup.find_all('loc'):
                urls.append(loc.string)
            logging.info(f"Successfully fetched {len(urls)} URLs from sitemap: {sitemap_url}")
            return urls
                
        except Exception as e:
            logging.warning(f"Error fetching or parsing sitemap {sitemap_url}: {e}. Falling back to local docs.")

    if local_docs_path and os.path.exists(local_docs_path):
        logging.info(f"Reading from local docs path: {local_docs_path}")
        for filepath in glob.glob(os.path.join(local_docs_path, '**/*.md'), recursive=True):
            urls.append(os.path.abspath(filepath))
        logging.info(f"Found {len(urls)} local markdown files.")
    elif local_docs_path:
        logging.warning(f"Local docs path '{local_docs_path}' does not exist.")
            
    return urls

def extract_text_from_source(source: str) -> str:
    if source.startswith('http://') or source.startswith('https://'):
        try:
            response = requests.get(source)
            response.raise_for_status()
            soup = BeautifulSoup(response.content, 'html.parser')
            main_content = soup.find('main') or soup.find('article') or soup.body
            if not main_content:
                logging.warning(f"No main content found for URL {source}")
                return ""
            for script_or_style in main_content(["script", "style"]):
                script_or_style.decompose()
            text = main_content.get_text(separator=' ', strip=True)
            return text
        except Exception as e:
            logging.error(f"Error fetching/parsing URL {source}: {e}")
            return ""
    else:
        try:
            with open(source, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            logging.error(f"Error reading local file {source}: {e}")
            return ""

def chunk_text(text: str) -> list[str]:
    # This is a simplified chunking strategy. More sophisticated methods exist.
    return [text[i:i + MAX_CHUNK_SIZE_CHARS] for i in range(0, len(text), MAX_CHUNK_SIZE_CHARS - OVERLAP_CHARS)]

def embed_texts(texts: list[str], input_type: str) -> list[list[float]]:
    co = get_cohere_client() # Now uses global env var
    try:
        response = co.embed(model=EMBEDDING_MODEL, texts=texts, input_type=input_type)
        return response.embeddings
    except Exception as e:
        logging.error(f"Cohere API error during embedding: {e}")
        return []

# --- RAG Agent Initialization (reusable) ---
# Initialize the RAG Agent once globally to avoid re-initializing on every request
rag_agent_instance = Agent( # Use Agent from agent.py
    name="RoboticsExpert",
    instructions="You are a helpful AI assistant specialized in Physical AI and Humanoid Robotics. Answer questions based on the provided tools and retrieved information. Only use the retrieve_from_qdrant tool to get information from the knowledge base.",
    tools=[retrieve_from_qdrant], # Access the function from the imported module
    model="gpt-4-0613", # Specify model for the agent
)

async def invoke_rag_agent(query: str) -> str:
    """Invokes the RAG agent with a query and returns its response asynchronously."""
    try:
        logging.info(f"Invoking RAG agent with query: {query}")
        result = await Runner.run(rag_agent_instance, query) # Use async Runner.run
        return result.final_output
    except Exception as e:
        logging.error(f"Error invoking RAG agent: {e}")
        return "An error occurred while communicating with the agent."

# --- Ingestion Logic (CLI) ---
cli_app = typer.Typer() # Keep typer for ingestion CLI

@cli_app.command()
def ingest():
    logging.info("RAG Ingestion Pipeline Started")
    
    qdrant_client = get_qdrant_client() # Now uses global env var
    
    try:
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(size=EMBEDDING_DIMENSION, distance=models.Distance.COSINE),
        )
        logging.info(f"Collection '{QDRANT_COLLECTION_NAME}' recreated successfully.")
    except Exception as e:
        logging.error(f"Error creating collection: {e}")
        return

    sitemap_url = "https://ai-robotic-text-book.vercel.app/sitemap.xml"
    local_docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'docs'))

    sources = get_all_urls(sitemap_url=sitemap_url, local_docs_path=local_docs_path)
    if not sources:
        logging.error("No URLs or local docs found. Exiting.")
        return

    all_chunks_data = []
    for source in sources:
        logging.info(f"Processing source: {source}")
        content = extract_text_from_source(source)
        if content:
            chunks = chunk_text(content)
            for chunk in chunks:
                all_chunks_data.append({"text": chunk, "source": source})
        else:
            logging.warning(f"Skipping source {source} due to empty content.")

    logging.info(f"Total chunks generated: {len(all_chunks_data)}")

    if all_chunks_data:
        texts_to_embed = [item["text"] for item in all_chunks_data]
        embeddings = embed_texts(texts_to_embed, input_type="search_document")
        
        if embeddings and len(all_chunks_data) == len(embeddings):
            points_to_upsert = []
            for i, item in enumerate(all_chunks_data):
                point_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{item['source']}::{item['text'][:50]}"))
                metadata = {"source_url": item['source'], "text_snippet": item['text'][:200]}
                points_to_upsert.append(models.PointStruct(id=point_id, vector=embeddings[i], payload=metadata))

            try:
                qdrant_client.upsert(collection_name=QDRANT_COLLECTION_NAME, points=points_to_upsert, wait=True)
                logging.info(f"Successfully upserted {len(points_to_upsert)} points.")
            except Exception as e:
                logging.error(f"Error upserting points to Qdrant: {e}")
        else:
            logging.error("Embedding failed or mismatch in chunk/embedding count.")

# --- Chat Endpoint ---
@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    logging.info(f"Received chat query: {request.query}")
    try:
        agent_response = await invoke_rag_agent(request.query) # Await the async function
        return ChatResponse(response=agent_response)
    except Exception as e:
        logging.error(f"Error in /chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

# --- Retrieval API ---
@app.post("/retrieve", response_model=List[RetrievalResult])
async def retrieve(request: QueryRequest):
    logging.info(f"Received query: {request.query}")
    try:
        qdrant_client = get_qdrant_client() # Now uses global env var
        
        query_embedding = embed_texts([request.query], input_type="search_query")[0]
        
        if not query_embedding:
            raise HTTPException(status_code=500, detail="Failed to embed query.")

        search_result = qdrant_client.query_points(
            collection_name=QDRANT_COLLECTION_NAME,
            query=query_embedding,
            limit=request.top_k,
            score_threshold=request.score_threshold
        )
        
        results = [
            RetrievalResult(text=hit.payload.get("text_snippet", ""), score=hit.score, metadata=hit.payload)
            for hit in search_result.points
        ]
        return results

    except Exception as e:
        logging.error(f"An error occurred during retrieval: {e}")
        raise HTTPException(status_code=500, detail="An internal error occurred during retrieval.")


if __name__ == "__main__":
    cli_app()