import os
import openai
import cohere
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
import logging
from openai import OpenAI
import json
import sys

# Import components from openai-agents SDK
from agents import Agent, Runner, function_tool

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load environment variables - keep for standalone script use
load_dotenv()

# --- Constants ---
EMBEDDING_MODEL = "embed-english-light-v3.0"
QDRANT_COLLECTION_NAME = "reg_embedding"

# --- Client Getter Functions (Modified to accept API keys) ---
def get_cohere_client(cohere_api_key: str) -> cohere.Client:
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY not found. Please set it in your .env file.")
    return cohere.Client(cohere_api_key)

def get_qdrant_client(qdrant_url: str, qdrant_api_key: str) -> QdrantClient:
    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY not found. Please set them in your .env file.")
    return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

def get_openai_client(openai_api_key: str) -> OpenAI:
    if not openai_api_key:
        raise ValueError("OPENAI_API_KEY not found. Please set it in your .env file.")
    return OpenAI(api_key=openai_api_key)

# --- Function Tools ---
@function_tool
def retrieve_from_qdrant(query: str, top_k: int = 5, score_threshold: float = 0.0) -> str:
    """
    Retrieves relevant text chunks from Qdrant based on a natural language query.
    This function acts as a tool for the AI agent.
    """
    logging.info(f"Retrieval tool called with query: '{query}', top_k: {top_k}, score_threshold: {score_threshold}")
    
    # Get API keys from environment (assuming they are loaded by the caller)
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_URL = os.getenv("QDRANT_URL")

    try:
        cohere_client_instance = get_cohere_client(COHERE_API_KEY)
        qdrant_client_instance = get_qdrant_client(QDRANT_URL, QDRANT_API_KEY)

        query_embedding = cohere_client_instance.embed(
            texts=[query],
            model=EMBEDDING_MODEL,
            input_type="search_query"
        ).embeddings[0]

        search_result = qdrant_client_instance.query_points(
            collection_name=QDRANT_COLLECTION_NAME,
            query=query_embedding,
            limit=top_k,
            score_threshold=score_threshold
        )
        
        if not search_result.points:
            logging.info("No relevant results found by Qdrant retrieval tool.")
            return "No relevant information found in the knowledge base."

        retrieved_texts = [hit.payload.get('text_snippet', '') for hit in search_result.points]
        
        return "\n\n".join(retrieved_texts)

    except Exception as e:
        logging.error(f"Error during Qdrant retrieval: {e}")
        return "An error occurred while retrieving information."

# Define the retrieval tool for the OpenAI agent
retrieval_tool = {
    "type": "function",
    "function": {
        "name": "retrieve_from_qdrant",
        "description": "Retrieves relevant text chunks from the knowledge base using a natural language query.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The natural language query for retrieval."
                },
                "top_k": {
                    "type": "integer",
                    "description": "The maximum number of relevant chunks to retrieve.",
                    "default": 5
                },
                "score_threshold": {
                    "type": "number",
                    "description": "The minimum relevance score for retrieved chunks.",
                    "default": 0.0
                }
            },
            "required": ["query"]
        }
    }
}

# --- Agent Main Function ---
def main(user_query: str):
    """
    Main function to run the AI agent.
    """
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY") # Ensure API key is loaded for agent init
    openai_client_instance = get_openai_client(OPENAI_API_KEY)

    agent = Agent(
        name="RoboticsExpert",
        instructions="You are a helpful AI assistant specialized in Physical AI and Humanoid Robotics. Answer questions based on the provided tools and retrieved information. Only use the retrieve_from_qdrant tool to get information from the knowledge base.",
        tools=[retrieve_from_qdrant],
        model="gpt-4-0613",
    )
    
    messages = [
        {"role": "system", "content": "You are a helpful AI assistant specialized in Physical AI and Humanoid Robotics. Answer questions based on the provided tools and retrieved information."},
        {"role": "user", "content": user_query}
    ]
    tools = [retrieval_tool]
    
    try:
        logging.info(f"Agent received query: {user_query}")
        result = Runner.run_sync(agent, user_query)
        print(result.final_output)

    except openai.APIError as e:
        logging.error(f"OpenAI API Error: {e}")
        print("An error occurred with the OpenAI API. Please check your API key and network connection.")
    except Exception as e:
        logging.error(f"An unexpected error occurred during agent interaction: {e}")
        print("An unexpected error occurred while processing your request.")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        user_query = " ".join(sys.argv[1:])
        main(user_query)
    else:
        print("Please provide a query as an argument. Example: python agent.py \"What is ROS?\"")
