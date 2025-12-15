# Research: Navigation Bar and Chatbot Widget

## Decision: Navigation Bar Modifications
**Rationale**: Need to modify the Docusaurus navigation configuration to remove "Blog" and "Chatbot" items from the navbar across all pages.
**Alternatives considered**:
- CSS-only hiding (not SEO-friendly, doesn't remove from DOM)
- JavaScript manipulation (not optimal for performance)

## Decision: Chatbot Widget Implementation
**Rationale**: Implement the chatbot as a floating action button (FAB) positioned at the bottom-right corner of the main page, using React component with fixed positioning and responsive CSS.
**Alternatives considered**:
- Full sidebar implementation (too intrusive)
- Top banner placement (would interfere with navigation)
- Modal popup (not always accessible)

## Decision: Page-Specific Display Logic
**Rationale**: Use React conditional rendering to only display the chatbot widget on the main/home page by checking the current route/path.
**Implementation approach**: Compare current page path with main page path to conditionally render the widget.

## Decision: Responsive Design Approach
**Rationale**: Use CSS media queries and responsive units to ensure the floating widget works well on both mobile and desktop devices.
**Considerations**:
- Mobile: Ensure widget doesn't overlap with mobile UI elements
- Desktop: Maintain appropriate spacing from viewport edges
- Accessibility: Maintain proper focus order and ARIA attributes

## Technology Stack Confirmation
- **Frontend Framework**: Docusaurus (React-based)
- **Styling**: CSS/SCSS with responsive design principles
- **Component Architecture**: React functional components with conditional rendering
- **Positioning**: CSS fixed positioning with bottom/right properties