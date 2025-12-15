# Data Model: Navigation Bar and Chatbot Widget

## UI State Entities

### Navigation Configuration
- **Entity**: NavbarConfig
- **Fields**:
  - items: Array of navigation items
  - visibleItems: Filtered list excluding "Blog" and "Chatbot"
- **Relationships**: Links to page routes
- **Validation**: Must maintain at least one navigation item after filtering

### Chatbot Widget State
- **Entity**: ChatbotWidgetState
- **Fields**:
  - isVisible: Boolean (true only on main page)
  - position: Object with x, y coordinates
  - isOpen: Boolean (chat interface open/closed)
  - isMinimized: Boolean (widget minimized/maximized)
- **State transitions**: hidden → visible → open → minimized
- **Validation**: Only one instance should exist per page

## Component Properties

### Navigation Bar Component
- **Props**:
  - currentPage: String (current route)
  - navigationItems: Array of navigation objects
- **Constraints**: Should not include "Blog" or "Chatbot" items

### Chatbot Widget Component
- **Props**:
  - position: String ("bottom-right" fixed position)
  - showOnPage: String (page where widget should appear)
  - isResponsive: Boolean (adapts to screen size)
- **Constraints**: Must be fixed positioned, responsive, and accessible

## UI Layout Specifications

### Responsive Breakpoints
- **Desktop**: >= 1024px width
- **Tablet**: 768px - 1023px width
- **Mobile**: < 768px width

### Positioning Rules
- **Floating Widget**: Fixed position at bottom-right corner
- **Offset**: 20px from bottom and right edges (adjustable for mobile)
- **Z-index**: Above other content but below modals