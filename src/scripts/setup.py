#!/usr/bin/env python3
"""
Setup script for initial database and collection creation
"""
import os
import sys
from pathlib import Path

# Add src to path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.config.vector_db import create_collection_if_not_exists
from src.config.database import Base, engine


def setup_databases():
    """
    Create required database tables and vector collections
    """
    print("Setting up databases...")
    
    # Create vector database collection
    print("Creating Qdrant collection...")
    create_collection_if_not_exists("documents")
    print("Qdrant collection created successfully!")
    
    # Create relational database tables
    print("Creating relational database tables...")
    try:
        # This would create all tables defined in models
        # In a real implementation, you'd use SQLAlchemy migrations
        Base.metadata.create_all(bind=engine)
        print("Relational database tables created successfully!")
    except Exception as e:
        print(f"Error creating relational database tables: {e}")
    
    print("Setup completed!")


if __name__ == "__main__":
    setup_databases()