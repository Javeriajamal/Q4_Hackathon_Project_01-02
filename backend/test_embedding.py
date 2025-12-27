#!/usr/bin/env python3
"""
Test script to verify embedding functionality
"""
import os
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize OpenAI client
openai_client = OpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1"
)

# Test embedding
try:
    print("Testing embedding generation...")
    response = openai_client.embeddings.create(
        model="BAAI/bge-large-en-v1.5",
        input=["What is the Physical AI & Humanoid Robotics book about?"]
    )

    embedding = response.data[0].embedding
    print(f"Embedding generated successfully!")
    print(f"Embedding dimension: {len(embedding)}")
    print(f"First 10 values: {embedding[:10]}")

except Exception as e:
    print(f"Error generating embedding: {e}")
    import traceback
    traceback.print_exc()