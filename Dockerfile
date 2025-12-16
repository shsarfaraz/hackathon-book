FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY book/my-book/backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application code
COPY book/my-book/backend/server.py .

# Expose the port (Railway will set PORT environment variable)
EXPOSE $PORT

# Run the application using uvicorn for production
CMD ["sh", "-c", "uvicorn server:app --host 0.0.0.0 --port $PORT"]