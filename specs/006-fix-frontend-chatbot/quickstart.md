# Quickstart: Fix Frontend Chatbot Response Issues

**Feature**: Fix Frontend Chatbot Response Issues
**Date**: 2025-12-17

## Overview
This guide provides quick setup and implementation instructions for improving the frontend chatbot's response accuracy by enhancing the client-side search algorithm.

## Prerequisites
- Node.js and npm installed
- Docusaurus development environment set up
- Access to the project codebase

## Implementation Steps

### 1. Locate the Chatbot Component
- File: `book/my-book/src/components/Chatbot/ChatbotWidget.js`
- Focus on the `searchBook` function (lines 49-69)

### 2. Replace Current Search Algorithm
Replace the current simple substring matching with a token-based relevance scoring algorithm:

```javascript
const searchBook = (query) => {
  if (!bookIndex || bookIndex.length === 0) {
    return "Sorry, the book content is not available for searching right now.";
  }

  // Tokenize the query into individual words
  const queryTokens = query.toLowerCase().split(/\s+/).filter(token => token.length > 0);

  if (queryTokens.length === 0) {
    return "Please enter a query to search the book content.";
  }

  // Calculate relevance scores for each book entry
  const scoredResults = bookIndex.map(entry => {
    let score = 0;
    const matchedTokens = [];

    // Score based on matches in chapter, title, and content
    queryTokens.forEach(token => {
      // Higher weight for chapter matches
      if (entry.chapter.toLowerCase().includes(token)) {
        score += 3;
        matchedTokens.push(token);
      }

      // Medium weight for title matches
      if (entry.title.toLowerCase().includes(token)) {
        score += 2;
        matchedTokens.push(token);
      }

      // Lower weight for content matches
      if (entry.content.toLowerCase().includes(token)) {
        score += 1;
        matchedTokens.push(token);
      }
    });

    return {
      entry,
      score,
      matchedTokens: [...new Set(matchedTokens)] // Remove duplicates
    };
  });

  // Filter to only entries with matches and sort by score (descending)
  const results = scoredResults
    .filter(item => item.score > 0)
    .sort((a, b) => b.score - a.score);

  if (results.length > 0) {
    // Return the top 3 results
    const topResults = results.slice(0, 3);
    return topResults.map(item =>
      `${item.entry.chapter}: ${item.entry.title}. ${item.entry.content}`
    ).join('\n\n');
  } else {
    return "Sorry, I couldn't find anything related to your question in the book. Try asking about ROS, Gazebo, Isaac Sim, or Humanoid Robotics.";
  }
};
```

### 3. Test the Implementation
1. Start the Docusaurus development server: `cd book/my-book && npm run start`
2. Open the website and test various queries about book topics
3. Verify that relevant content is returned for ROS, Gazebo, Isaac Sim, and Humanoid Robotics queries

### 4. Verify Success Criteria
- 90% of technical queries return relevant content
- Improved response quality for target topics
- Reduced fallback responses
- Fast response times (under 2 seconds)

## Deployment
When ready for production:
1. Build the site: `npm run build`
2. Deploy to hosting platform (e.g., Vercel)
3. Verify functionality on production site