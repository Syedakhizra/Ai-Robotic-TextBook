import os
import typer
import cohere
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load environment variables
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")

# Initialize clients
try:
    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY not found. Please set it in your .env file.")
    cohere_client = cohere.Client(COHERE_API_KEY)
    
    if not QDRANT_URL or not QDRANT_API_KEY:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY not found. Please set them in your .env file.")
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
except Exception as e:
    logging.error(f"Error initializing clients: {e}")
    exit(1)

# Create a Typer app
app = typer.Typer()

@app.command()
def retrieve(
    query: str,
    top_k: int = typer.Option(5, "--top_k", "-k"),
    score_threshold: float = typer.Option(0.0, "--score_threshold", "-s"), # Temporarily set to 0.0 for debugging
):
    """
    Retrieves and validates the RAG retrieval pipeline.
    """
    logging.info(f"Retrieving results for query: '{query}'")

    try:
        # Embed the query
        logging.info("Embedding the query...")
        query_embedding = cohere_client.embed(
            texts=[query],
            model="embed-english-light-v3.0",
            input_type="search_query"
        ).embeddings[0]

        # Perform semantic retrieval using query_points
        logging.info("Performing semantic retrieval using query_points in Qdrant...")
        
        search_result = qdrant_client.query_points(
            collection_name="reg_embedding",
            query=query_embedding,
            limit=top_k,
            score_threshold=score_threshold
        )
        
        # Process and print results
        logging.info("Processing and printing results...")
        if not search_result.points:
            print("No relevant results found.")
            return

        for hit in search_result.points:
            print(f"Score: {hit.score}")
            print(f"Text: {hit.payload.get('text_snippet', '')}")
            print(f"Metadata: {hit.payload}")
            print("-" * 20)

    except Exception as e:
        logging.error(f"An error occurred during retrieval: {e}")

if __name__ == "__main__":
    app()