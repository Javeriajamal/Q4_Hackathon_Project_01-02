# Quickstart: Physical AI And Humanoid Robotics Book Website

## Development Setup

### Prerequisites
- Node.js (LTS version recommended)
- npm or yarn package manager
- Git

### Installation Steps

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies**
   ```bash
   cd website
   npm install
   # or
   yarn install
   ```

3. **Start development server**
   ```bash
   npm run dev
   # or
   yarn dev
   ```
   The site will be available at http://localhost:3000

### Project Structure Overview
```
website/
├── docusaurus.config.js     # Main Docusaurus configuration
├── src/
│   ├── components/         # Custom React components
│   ├── pages/              # Custom pages (Home, About, Contact)
│   └── css/                # Custom styles
├── docs/                   # Book content in MDX format
└── static/                 # Static assets
```

### Key Configuration Files

- `docusaurus.config.js`: Site metadata, navigation, plugins
- `src/css/custom.css`: Custom styles and Tailwind configuration
- `src/theme/`: Custom theme components for layout and navigation

## Development Workflow

### Adding New Book Content
1. Create new MDX files in `docs/book/chapter-X/`
2. Add content using MDX syntax with embedded React components
3. Update sidebar configuration in `docusaurus.config.js`

### Customizing Components
1. Create new components in `src/components/`
2. Override default theme components in `src/theme/`
3. Use Tailwind CSS classes for styling

### Testing Changes
1. Run `npm run dev` to start development server
2. Make changes to components/content
3. View changes in real-time at http://localhost:3000
4. Run `npm run build` to build for production

## Deployment

### Building for Production
```bash
npm run build
```

### Deployment Options
- GitHub Pages (built-in Docusaurus support)
- Vercel
- Netlify
- Any static hosting service

## Key Commands

- `npm run dev` - Start development server
- `npm run build` - Build for production
- `npm run serve` - Serve built site locally
- `npm run deploy` - Deploy to configured platform