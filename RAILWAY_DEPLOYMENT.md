# Deploying Your Backend to Railway

## Overview
This guide will help you deploy your Python FastAPI backend to Railway. Your backend is a RAG (Retrieval-Augmented Generation) system that processes documents and provides chat functionality using Cohere AI and Qdrant vector database.

## Prerequisites
1. Railway account (sign up at https://railway.app)
2. API keys for:
   - Qdrant vector database (QDRANT_API_KEY and QDRANT_URL)
   - Cohere AI service (COHERE_API_KEY)

## Deployment Steps

### Option 1: Deploy Directly from GitHub (Recommended)
1. Connect your Railway account to your GitHub
2. Create a new project and select your repository
3. Railway will automatically detect the Dockerfile and deploy your application
4. Add the required environment variables in the next step

### Option 2: Deploy Using Railway CLI
1. Install Railway CLI: `npm install -g @railway/cli`
2. Login: `railway login`
3. Link your project: `railway link`
4. Set variables: `railway up`

## Environment Variables Required
Add these environment variables in the Railway dashboard under "Variables":

- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_URL`: Your Qdrant database URL
- `COHERE_API_KEY`: Your Cohere API key

Optional variables:
- `TARGET_URL`: URL to crawl for the embedding pipeline (default: https://hackathon-book-sigma.vercel.app/)

## API Endpoints
Once deployed, your backend will be available at:
- Health check: `https://<your-app-name>.up.railway.app/health`
- Chat endpoint: `https://<your-app-name>.up.railway.app/api/chat`
- Root endpoint: `https://<your-app-name>.up.railway.app/`

## Configuration Notes
- The application will run on the port specified by the `$PORT` environment variable (set by Railway)
- The Dockerfile uses uvicorn to serve the FastAPI application
- CORS is configured to allow all origins (you may want to restrict this in production)

## Running the Embedding Pipeline
If you need to run the embedding pipeline after deployment:
1. SSH into your Railway instance: `railway ssh`
2. Run the pipeline: `python server.py --url <target-url> --max-depth <depth>`

## Troubleshooting
- Check the logs in the Railway dashboard if the application fails to start
- Ensure all required environment variables are set
- Verify that your Qdrant and Cohere API keys are valid and have the necessary permissions

## Updating Your Deployment
When you push changes to your GitHub repository, Railway will automatically redeploy your application if you chose the GitHub integration option.