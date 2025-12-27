"""
RAG Chatbot API Server
Handles queries by embedding with Qwen model via OpenRouter, retrieves from Qdrant, and generates responses
"""

import os
import re
import logging
from typing import List

from openai import OpenAI
from qdrant_client import QdrantClient
from dotenv import load_dotenv

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

print(os.getenv("OPENROUTER_API_KEY"))


# -------------------------
# Load environment variables
# -------------------------
load_dotenv()

# -------------------------
# Logging
# -------------------------
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# -------------------------
# Config
# -------------------------
def load_config():
    return {
        "openrouter_api_key": os.getenv("OPENROUTER_API_KEY"),  # Use API key from environment
        "qdrant_url": os.getenv("QDRANT_URL"),
        "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
        "collection_name": os.getenv("COLLECTION_NAME", "rag_embedding"),
    }

# -------------------------
# Qdrant
# -------------------------
def get_qdrant_client():
    cfg = load_config()
    return QdrantClient(
        url=cfg["qdrant_url"],
        api_key=cfg["qdrant_api_key"]
    )

def search_qdrant(query_embedding: List[float], limit: int = 3):
    cfg = load_config()
    client = get_qdrant_client()

    results = client.query_points(
        collection_name=cfg["collection_name"],
        query=query_embedding,
        limit=limit
    )

    # Access the points from the QueryResponse object
    return [r.payload.get("text", "") for r in results.points]

# -------------------------
# FastAPI app
# -------------------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------
# Models
# -------------------------
class ChatRequest(BaseModel):
    query: str
    session_id: str | None = None

# -------------------------
# Health
# -------------------------
@app.get("/api/health")
def health():
    return {"status": "ok"}

# -------------------------
# Chat endpoint
# -------------------------
@app.post("/api/chat")
def chat(req: ChatRequest, request: Request):
    logger.info(f"Received chat request: {req.dict()}")

    cfg = load_config()
    openai_client = OpenAI(
        api_key=cfg["openrouter_api_key"],
        base_url="https://openrouter.ai/api/v1"
    )

    try:
        # 🔹 Embed query using BGE embedding model
        embed = openai_client.embeddings.create(
            model="BAAI/bge-large-en-v1.5",
            input=[req.query]
        )
        query_embedding = embed.data[0].embedding

        # 🔹 Retrieve context from Qdrant
        contexts = search_qdrant(query_embedding)
        context = "\n".join(contexts) or "No relevant context found."

    except Exception as e:
        logger.error(f"Qdrant error: {e}")
        context = "No context available."

    try:
        # 🔹 Chat with OpenRouter using a suitable model for question answering
        resp = openai_client.chat.completions.create(
            model="microsoft/wizardlm-2-8x22b",  # Use a reliable model available on OpenRouter
            messages=[
                {
                    "role": "system",
                    "content": "You are a helpful AI assistant for the Physical AI & Humanoid Robotics book. Answer questions based only on the provided context."
                },
                {
                    "role": "user",
                    "content": f"""
Answer ONLY using the content below.

CONTENT:
{context}

QUESTION:
{req.query}

ANSWER:
"""
                }
            ],
            max_tokens=250,
        )

        answer = resp.choices[0].message.content.strip()

    except Exception as e:
        logger.error(f"OpenRouter chat error: {e}")
        answer = "Sorry, I could not generate a response."

    return {
        "response": answer,
        "session_id": req.session_id,
        "source_attributions": []
    }

# -------------------------
# Run
# -------------------------
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
