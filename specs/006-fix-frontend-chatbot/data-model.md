# Data Model: Improved Chatbot Search

**Feature**: Fix Frontend Chatbot Response Issues
**Date**: 2025-12-17

## Entities

### BookContentEntry
Represents a single indexed entry from the book

- **id**: string (optional, for tracking)
- **chapter**: string - The module/chapter name (e.g., "Module 1: ROS 2 Core Concepts")
- **title**: string - The specific topic title (e.g., "ROS 2 Nodes")
- **content**: string - The detailed content for that topic
- **keywords**: string[] (derived) - Extracted keywords for improved matching

### SearchQuery
Represents a user's search input

- **text**: string - The raw query text from user
- **tokens**: string[] - Individual words from the query for matching
- **options**: object - Search options (e.g., max_results, match_threshold)

### SearchResult
Represents a single search result with relevance information

- **entry**: BookContentEntry - The matching book content
- **relevance_score**: number - Calculated relevance score (0-1)
- **matched_tokens**: string[] - Which query tokens were found
- **match_details**: object - Details about where matches occurred (chapter, title, content)

### SearchResultsCollection
Container for multiple search results

- **query**: SearchQuery - The original query
- **results**: SearchResult[] - Array of ranked results
- **total_count**: number - Total number of matches found
- **execution_time**: number - Time taken for search in milliseconds