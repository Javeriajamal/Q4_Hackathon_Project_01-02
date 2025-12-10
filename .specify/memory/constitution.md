# Constitution: Physical AI And Humanoid Robotics Book

## Project Overview
A modern, accessible book about Physical AI and Humanoid Robotics built with cutting-edge web technologies and designed for optimal user experience.

## Core Principles
- **Zero external UI libraries**: Except shadcn/ui & Tailwind for consistent, lightweight design
- **Mobile-first design**: Responsive layout optimized for all devices
- **Glassmorphism + gradient design**: Modern aesthetic with depth and visual appeal
- **MDX for content**: All book content uses MDX for easy future updates and flexibility
- **Dark/Light mode toggle**: User preference support with seamless theme switching
- **Lightning fast performance**: Target <1.5s First Contentful Paint (FCP)
- **100% accessibility**: Full ARIA compliance and comprehensive keyboard navigation
- **Standards compliance**: Follow web standards and best practices

## Technical Standards
- **TypeScript strict mode**: All code written with strict type checking enabled
- **Reusable components**: All UI components designed for maximum reusability
- **Content organization**: Chapter content stored in `/content/chapters/`
- **Bundle size optimization**: Total bundle size maintained under 150KB (excluding images)

## Project Constraints
- **Limited pages**: Only 4 pages - Home, Book, About, Contact
- **Book structure**: 5 chapters × 2 topics each for focused content delivery
- **Deployment**: Hosted on Vercel (free tier) for cost-effective distribution
- **Performance budget**: Aggressive optimization to maintain fast load times

## Architecture Guidelines
- **Component design**: Modular, composable components following atomic design principles
- **State management**: Efficient state handling with minimal re-renders
- **Accessibility first**: Keyboard navigation, screen reader support, and focus management from inception
- **Responsive design**: Mobile-first approach with progressive enhancement

## Quality Assurance
- **Performance monitoring**: Regular audits to maintain <1.5s FCP
- **Accessibility testing**: WCAG 2.1 AA compliance verification
- **Cross-browser compatibility**: Support for modern browsers (last 2 versions)
- **Bundle analysis**: Continuous monitoring of bundle size to stay under 150KB limit

## Content Strategy
- **MDX integration**: Leverage MDX for rich, interactive book content
- **Structured chapters**: Organized in 5 chapters with 2 topics each for optimal learning progression
- **Future extensibility**: Easy content updates and additions through MDX format
- **Media optimization**: Proper handling of images and media for performance

## Deployment & Operations
- **Vercel hosting**: Leveraging Vercel's edge network for global performance
- **CDN optimization**: Efficient asset delivery and caching strategies
- **SEO considerations**: Proper meta tags, structured data, and crawlability
- **Analytics**: Performance and user experience monitoring

## Success Metrics
- Page load time < 1.5s FCP
- Bundle size < 150KB
- Accessibility score 100%
- Mobile responsiveness across all breakpoints
- Successful theme switching without jank
- User-friendly navigation and content consumption experience