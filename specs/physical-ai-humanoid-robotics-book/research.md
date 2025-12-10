# Research: Physical AI And Humanoid Robotics Book Website

## Phase 0: Technical Research and Decisions

### Docusaurus vs Next.js Decision

**Decision**: Use Docusaurus for the book website
**Rationale**:
- Docusaurus is purpose-built for documentation sites with excellent MDX support
- Built-in features like sidebar navigation, search, and versioning
- Strong theming capabilities for custom UI
- Optimized for content-heavy sites like books
- Better SEO and accessibility out of the box
- Simpler deployment and maintenance compared to custom Next.js solution

**Alternatives considered**:
- Next.js with App Router: More complex setup, requires building navigation from scratch
- Gatsby: Good alternative but Docusaurus has better documentation-specific features
- Vanilla React: Would require building everything from scratch

### MDX vs Plain React Decision

**Decision**: Use MDX for all book content (as specified in requirements)
**Rationale**:
- Easy content updates without code changes
- Markdown familiarity for authors
- React component embedding capability
- Built-in syntax highlighting
- Docusaurus native support for MDX

### Theme System Decision

**Decision**: Implement dark mode using Docusaurus theme system with custom CSS
**Rationale**:
- Docusaurus has built-in dark mode support
- Easy to toggle between themes
- Consistent across all pages
- Respects user system preferences

### Navigation and Progress Tracking

**Decision**: Implement custom navigation and progress tracking using Docusaurus plugins
**Rationale**:
- Docusaurus allows custom theme components for navigation
- Can implement progress tracking with scroll detection
- MDX content can be enhanced with custom components for progress indicators
- localStorage for persisting reading progress

### LocalStorage vs Backend Decision

**Decision**: Use localStorage for waitlist persistence (as specified in requirements)
**Rationale**:
- No backend infrastructure required
- Client-side only as per requirements
- Sufficient for MVP waitlist functionality
- Simple implementation and maintenance

### Mobile Responsiveness

**Decision**: Use Docusaurus responsive design with Tailwind CSS
**Rationale**:
- Docusaurus is mobile-responsive by default
- Tailwind CSS provides utility classes for responsive design
- Easy to customize for mobile reading experience
- Consistent across all devices