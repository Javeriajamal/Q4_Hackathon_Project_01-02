import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI and Humanoid Robotics',
      collapsible: false,
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Introduction to Physical AI',
          items: [
            'book/chapter-1/foundations-and-principles',
            'book/chapter-1/current-state-and-applications'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Humanoid Robotics Fundamentals',
          items: [
            'book/chapter-2/design-and-mechanics',
            'book/chapter-2/control-systems-and-actuation'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: AI Integration in Robotics',
          items: [
            'book/chapter-3/perception-and-sensing',
            'book/chapter-3/decision-making-and-learning'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Applications and Use Cases',
          items: [
            'book/chapter-4/industrial-and-service-robotics',
            'book/chapter-4/healthcare-and-social-robotics'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Future Directions and Ethics',
          items: [
            'book/chapter-5/emerging-technologies-and-trends',
            'book/chapter-5/ethical-considerations-and-societal-impact'
          ],
        },
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
