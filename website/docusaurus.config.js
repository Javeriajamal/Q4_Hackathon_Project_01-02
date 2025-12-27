// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to the intersection of artificial intelligence and physical systems',
  favicon: 'img/humanoid-logo.png',

  // ✅ Use localhost for development
  url: 'https://javeriajamal.github.io',
  baseUrl: '/',

  organizationName: 'Javeria jamal',
  projectName: 'physical-ai-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs', // all docs under /docs
          // ✅ If you want math support in future:
          // remarkPlugins: [require('remark-math')],
          // rehypePlugins: [require('rehype-katex')],
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themes: [
    // Optional: enable Mermaid diagrams if needed
    // '@docusaurus/theme-mermaid',
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.0/dist/katex.min.css',
      type: 'text/css',
      /*integrity: 'sha384-Xi8rHCmBmhbuyyhbI88391ZKP2dmfnOl4umQbY6jYiZq14wTIFTGi8fA1o3ic5C',*/
      crossorigin: 'anonymous',
    },
  ],

  themeConfig: /** @type {import('@docusaurus/preset-classic').ThemeConfig} */ ({
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/humanoid-logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/Javeriajamal',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Book',
              to: '/docs/module-1-ros-2/chapter-1-introduction-to-ros2',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Sign Up',
              to: '/contact',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Javeriajamal',
            },
          ],
        }
      ],
      copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash'],
    },
  }),
};

export default config;
