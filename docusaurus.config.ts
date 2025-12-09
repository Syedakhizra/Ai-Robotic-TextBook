import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A complete AI-native textbook + Docusaurus site for Physical AI & Humanoid Robotics',
  favicon: 'assets/img/favicon.ico', // Changed favicon path to assets/img
  staticDirectories: ['assets'],

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Syedakhizra.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Syedakhizra', // Usually your GitHub org/user name.
  projectName: 'ai-robotics-textbook', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Syedakhizra/ai-robotics-textbook/tree/main/',
        },

        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: '/img/docusaurus-social-card.jpg', // Updated path
    colorMode: {
      defaultMode: 'dark', // Changed to dark
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: 'Physical AI & HR',
      logo: {
        alt: 'AI Robotics Textbook Logo',
        src: '/img/logo.svg', // Corrected path
        srcDark: '/img/logo_dark.svg', // Corrected path
      },
            items: [
              {
                type: 'docSidebar',
                sidebarId: 'bookSidebar', // Updated sidebarId
                position: 'left',
                label: 'Docs', // Changed from Tutorial to Docs
              },
      
              {
                href: 'https://github.com/Syedakhizra/Ai-Robotic-TextBook.git',
                label: 'GitHub',
                position: 'right',
              },
            ],
          },
          footer: {
            style: 'dark', // Keep footer dark
            links: [
              {
                title: 'Docs',
                items: [
                  {
                    label: 'Docs', // Changed from Tutorial to Docs
                    to: '/docs', // Updated link
                  },
                ],
              },
              {
                title: 'Community',
                items: [
            {
              label: 'X',
              href: 'https://x.com/syedakhizra00?s=11',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/in/syeda-khizra-hussain-5826262b9',
            },
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Syedakhizra/Ai-Robotic-TextBook.git',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Textbook Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
