# Implementation Plan: Navigation Bar and Chatbot Widget

**Branch**: `001-nav-chatbot-widget` | **Date**: 2025-12-15 | **Spec**: N:/Documents/Sarfaraz/Generative AI/add_agent_in_hackathon/hackathon/specs/001-nav-chatbot-widget/spec.md
**Input**: Feature specification from `/specs/001-nav-chatbot-widget/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of UI changes to remove "Blog" and "Chatbot" items from the navigation bar, configure the chatbot as a floating widget positioned at the bottom-right corner of the main page only, with fixed positioning during scrolling and responsive behavior across mobile and desktop devices.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: TypeScript/JavaScript (Docusaurus-based site)  
**Primary Dependencies**: Docusaurus framework, React components, CSS/SCSS styling  
**Storage**: N/A (UI changes only)  
**Testing**: Jest for component testing, manual UI testing  
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Web  
**Performance Goals**: No impact on page load time (>0.1 seconds increase), smooth scrolling with fixed widget  
**Constraints**: Must maintain existing functionality for other navigation items, accessible design standards  
**Scale/Scope**: Single website with responsive layout for mobile and desktop

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Alignment Verification**:
- **Accuracy**: Verify all responses will be grounded solely in the book's content or user-selected text only
- **Efficiency**: Confirm use of free-tier services (Qdrant Cloud Free, Neon Serverless Postgres) without exceeding limits
- **Modularity**: Ensure clean separation of concerns for easy maintenance, testing, and future extensions
- **User-centric Design**: Validate seamless handling of general book queries and isolated queries on user-selected text
- **Security & Privacy**: Confirm secure handling of Cohere API keys, no unnecessary data storage, and prevention of content leakage
- **LLM Provider Standard**: Verify Cohere API used exclusively for both embeddings and generation (no OpenAI allowed)
- **Response Grounding**: Ensure every generated response will be traceable to specific retrieved chunks from the book
- **Code Quality**: Confirm PEP8 compliance, type hints, comprehensive docstrings, and modular structure
- **Testing Requirements**: Verify minimum 80% recall on retrieval tests, unit tests, and integration tests will be included
- **Performance Goals**: Confirm target average response time ≤ 5 seconds will be achievable
- **Tech Stack Compliance**: Validate the use of Cohere API, FastAPI, Neon Postgres, and Qdrant Cloud Free Tier
- **Deployment Constraints**: Ensure solution will be embeddable in a published digital book

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)```textbook/my-book/├── src/│   ├── components/│   │   └── Chatbot/│   │       ├── ChatbotWidget.js│   │       └── ChatbotWidget.module.css│   ├── pages/│   │   └── index.js (main page)│   └── css/│       └── custom.css (navbar modifications)├── docusaurus.config.js (navbar configuration)└── static/```**Structure Decision**: The feature will be implemented in the Docusaurus-based website located in the book/my-book directory. The navigation changes will be made in docusaurus.config.js, and the chatbot widget will be implemented as a React component in src/components/Chatbot/. The main page (index.js) will be updated to include conditional rendering logic for the widget.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
