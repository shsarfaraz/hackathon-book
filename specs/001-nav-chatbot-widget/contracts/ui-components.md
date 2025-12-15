# UI Component Contracts: Navigation Bar and Chatbot Widget

## Navigation Bar Component

### Interface
```typescript
interface NavbarConfig {
  items: NavbarItem[];
  className?: string;
}

interface NavbarItem {
  type: string;
  label: string;
  to?: string;
  href?: string;
}
```

### Behavior
- Renders navigation items in header
- Excludes "Blog" and "Chatbot" items per specification
- Responsive design adapts to screen size
- Maintains accessibility standards

## Chatbot Widget Component

### Interface
```typescript
interface ChatbotWidgetProps {
  position?: 'bottom-right';
  showOnPages?: string[];
  isOpen?: boolean;
  onToggle?: () => void;
}

interface ChatbotWidgetState {
  isVisible: boolean;
  isOpen: boolean;
  isMinimized: boolean;
}
```

### Behavior
- Fixed position at bottom-right corner of viewport
- Only renders on main/home page
- Toggle between minimized and expanded states
- Maintains position during scrolling
- Responsive design for mobile/desktop

## CSS Contract

### Required Classes
- `.chatbot-widget` - Main widget container
- `.chatbot-widget--fixed` - Fixed positioning styles
- `.chatbot-widget--hidden` - Hidden state
- `.chatbot-widget--open` - Open/expanded state

### Positioning Requirements
- Fixed position with bottom/right: 20px on desktop
- Adjusted positioning for mobile screens
- Z-index > 1000 to appear above other content
- Smooth transitions for open/close animations