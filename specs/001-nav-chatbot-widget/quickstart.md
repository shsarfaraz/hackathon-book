# Quickstart: Navigation Bar and Chatbot Widget

## Prerequisites
- Node.js 16+ installed
- Docusaurus project set up
- Access to the website's source code

## Implementation Steps

### 1. Update Navigation Configuration
1. Locate the navbar configuration file (typically `docusaurus.config.js` or similar)
2. Remove "Blog" and "Chatbot" items from the navigation array
3. Test that navigation renders correctly without these items

### 2. Create Chatbot Widget Component
1. Create a new React component for the floating chatbot widget
2. Implement fixed positioning at bottom-right corner
3. Add conditional rendering logic to show only on main page
4. Ensure responsive behavior for different screen sizes

### 3. Integrate Widget with Main Page
1. Import the chatbot widget component in the main page
2. Add conditional rendering based on current page
3. Verify widget appears only on main page

### 4. Test Responsive Behavior
1. Test on different screen sizes (desktop, tablet, mobile)
2. Verify widget positioning remains correct
3. Check that widget doesn't interfere with other UI elements

## File Locations
- **Navbar config**: `docusaurus.config.js` or `src/components/Navbar.js`
- **Chatbot widget**: `src/components/ChatbotWidget.js`
- **Main page**: `src/pages/index.js` or equivalent

## Key CSS Properties
- Position: `fixed`
- Bottom: `20px`
- Right: `20px`
- Z-index: `1000` (or higher than other content)
- Responsive units for mobile compatibility