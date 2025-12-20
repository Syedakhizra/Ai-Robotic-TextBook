import React, { JSX } from 'react';
import Layout from '@theme/Layout';
import HeroSection from '../components/HeroSection';
import FeaturesSection from '../components/FeaturesSection';
import ModulesSection from '../components/ModulesSection';
import CtaSection from '../components/CtaSection';

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Home"
      description="A complete AI-native textbook + Docusaurus site for Physical AI & Humanoid Robotics"
    >
      <HeroSection />
      <main>
        <FeaturesSection />
        <ModulesSection />
      </main>
      <CtaSection />
    </Layout>
  );
}
