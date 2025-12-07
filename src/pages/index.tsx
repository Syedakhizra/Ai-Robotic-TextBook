import React, { JSX, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { gsap } from "gsap";
import { ScrollTrigger } from "gsap/ScrollTrigger";

import styles from './index.module.css';

// Register GSAP plugins
gsap.registerPlugin(ScrollTrigger);

// Custom component for Animated Cards (used for Features and Module Overview)
function AnimatedCard({ Svg, title, description, id, extraContent }) {
  const cardRef = useRef(null);

  useEffect(() => {
    gsap.fromTo(
      cardRef.current,
      { y: 50, opacity: 0 },
      {
        y: 0,
        opacity: 1,
        duration: 0.8,
        ease: 'power3.out',
        scrollTrigger: {
          trigger: cardRef.current,
          start: 'top 80%',
          toggleActions: 'play none none none',
        },
      }
    );
  }, []);

  return (
    <div ref={cardRef} id={id} className={clsx('col col--4 margin-bottom--lg')}>
      <div className={clsx("card shadow--md", styles.featureCard)}>
        <div className="card__header text--center">
          {Svg && <Svg className={styles.featureSvg} role="img" />}
        </div>
        <div className="card__body text--center">
          <h3 className={styles.featureTitle}>{title}</h3>
          <p className={styles.featureDescription}>{description}</p>
        </div>
        {extraContent && (
          <div className="card__footer">
            <small>{extraContent}</small>
          </div>
        )}
      </div>
    </div>
  );
}

function HomepageHero() {
  const { siteConfig } = useDocusaurusContext();
  const heroRef = useRef(null);

  useEffect(() => {
    const tl = gsap.timeline();
    tl.fromTo(
      '.hero__title',
      { y: 50, opacity: 0 },
      { y: 0, opacity: 1, duration: 1, ease: 'power3.out' },
      '+=0.2'
    );
    tl.fromTo(
      '.hero__subtitle',
      { y: 50, opacity: 0 },
      { y: 0, opacity: 1, duration: 1, ease: 'power3.out' },
      '-=0.8'
    );
    tl.fromTo(
      '.buttons',
      { y: 50, opacity: 0 },
      { y: 0, opacity: 1, duration: 1, ease: 'power3.out' },
      '-=0.8'
    );
  }, []);

  return (
    <header ref={heroRef} className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className={clsx("hero__title", styles.heroTitle)}>{siteConfig.title}</h1>
        <p className={clsx("hero__subtitle", styles.heroSubtitle)}>{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className={clsx("button button--secondary button--lg", styles.heroButton)}
            to={useBaseUrl('/docs/preface')}>
            Start Reading 📖
          </Link>
          <Link
            className={clsx("button button--info button--lg", styles.heroButton)}
            to="https://github.com/Syedakhizra/ai-robotics-textbook">
            GitHub Repository 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  return (
    <Layout
      title={`Home`}
      description="A complete AI-native textbook and Docusaurus site for Physical AI & Humanoid Robotics">
      <HomepageHero />
      <main>
        {/* Features Section */}
        <section className={clsx(styles.section, styles.featuresSection)}>
          <div className="container">
            <h2 className={clsx("text--center", styles.sectionTitle)}>Key Features</h2>
            <div className="row">
              <AnimatedCard
                title="Foundational ROS 2"
                description="Master the Robot Operating System 2, the backbone of modern robotics software development."
                Svg={require('@site/assets/img/ros2-logo.svg').default}
                id="feature-ros2" extraContent={undefined}              />
              <AnimatedCard
                title="Digital Twin Simulation"
                description="Build and interact with virtual robots in high-fidelity environments using Gazebo and Unity."
                Svg={require('@site/assets/img/simulation-logo.svg').default}
                id="feature-simulation" extraContent={undefined}              />
              <AnimatedCard
                title="NVIDIA Isaac AI Stack"
                description="Explore advanced AI perception, simulation, and deployment with Isaac ROS and Isaac Sim."
                Svg={require('@site/assets/img/nvidia-logo.svg').default}
                id="feature-nvidia" extraContent={undefined}              />
              <AnimatedCard
                title="Vision-Language-Action (VLA)"
                description="Bridge human language, robot vision, and physical action with cutting-edge AI models."
                Svg={require('@site/assets/img/vla-logo.svg').default}
                id="feature-vla" extraContent={undefined}              />
              <AnimatedCard
                title="Autonomous Humanoids"
                description="Integrate all concepts into a capstone project for a voice-controlled, intelligent humanoid robot."
                Svg={require('@site/assets/img/humanoid-logo.svg').default}
                id="feature-humanoid" extraContent={undefined}              />
              <AnimatedCard
                title="AI-Native & Localized"
                description="Content structured for AI agents, supporting future personalization and Urdu translation."
                Svg={require('@site/assets/img/ai-native-logo.svg').default}
                id="feature-ai-native" extraContent={undefined}              />
               <AnimatedCard
                title="RAG Chatbot Ready"
                description="Prepare your learning material for seamless integration with advanced Retrieval-Augmented Generation chatbots."
                Svg={require('@site/assets/img/chatbot-logo.svg').default}
                id="feature-rag" extraContent={undefined}              />
              <AnimatedCard
                title="Reproducible Code Examples"
                description="Hands-on learning with verified code examples runnable on real or simulated hardware."
                Svg={require('@site/assets/img/code-logo.svg').default}
                id="feature-code" extraContent={undefined}              />
            </div>
          </div>
        </section>

        {/* Module Overview Section */}
        <section className={clsx(styles.section, styles.moduleOverviewSection, styles.altBackground)}>
          <div className="container">
            <h2 className={clsx("text--center", styles.sectionTitle)}>Book Structure & Module Overview</h2>
            <div className="row">
              <AnimatedCard
                title="Week 1-3: ROS 2 Foundations"
                description="Master core ROS 2 concepts: nodes, topics, services, actions, packages, launch files, and URDF basics for robot description."
                extraContent="Module 1"
                id="module-1" Svg={undefined}              />
              <AnimatedCard
                title="Week 4-6: Digital Twin Simulation"
                description="Build and interact with virtual robots using Gazebo and Unity, focusing on sensor simulation and URDF/SDF pipelines."
                extraContent="Module 2"
                id="module-2" Svg={undefined}              />
              <AnimatedCard
                title="Week 7-9: NVIDIA Isaac AI Stack"
                description="Explore Isaac Sim for high-fidelity simulation, Isaac ROS for accelerated perception, synthetic data, and Jetson deployment."
                extraContent="Module 3"
                id="module-3" Svg={undefined}              />
              <AnimatedCard
                title="Week 10-12: Vision-Language-Action (VLA)"
                description="Bridge human language and robot perception with LLMs, motion planning, and ROS integration for physical robot control."
                extraContent="Module 4"
                id="module-4" Svg={undefined}              />
              <AnimatedCard
                title="Week 13: Capstone Project"
                description="Integrate all learned concepts to build an autonomous humanoid robot pipeline, evaluated against a comprehensive rubric."
                extraContent="Capstone"
                id="module-capstone" Svg={undefined}              />
              <AnimatedCard
                title="Appendices"
                description="Essential supplementary materials: hardware requirements, installation guides, glossary, and comprehensive references."
                extraContent="Reference"
                id="module-appendices" Svg={undefined}              />
            </div>
          </div>
        </section>

        {/* Learning Outcomes Section */}
        <section className={clsx(styles.section, styles.learningOutcomesSection)}>
          <div className="container">
            <h2 className={clsx("text--center", styles.sectionTitle)}>What You Will Learn</h2>
            <div className="row">
              <ul className={clsx('col col--10 col--offset-1', styles.learningOutcomeList)}>
                <li>**Master ROS 2 Fundamentals**: Understand the backbone of modern robotics software development.</li>
                <li>**Build Digital Twins**: Learn to create and utilize high-fidelity simulations with Gazebo and Unity.</li>
                <li>**Explore NVIDIA Isaac AI Stack**: Dive into advanced AI perception, simulation, and deployment with Isaac ROS and Isaac Sim.</li>
                <li>**Integrate Vision-Language-Action Models**: Discover how to bridge natural language commands with robot perception and physical execution.</li>
                <li>**Design Autonomous Humanoids**: Apply all learned concepts to a challenging capstone project.</li>
                <li>**Ensure Reproducibility**: Develop with verified code examples runnable on real or simulated hardware.</li>
                <li>**Adopt AI-Native Practices**: Structure content for machine readability, personalization, and translation.</li>
                <li>**Practice Safety & Ethics**: Understand and apply principles of safe human-robot interaction.</li>
              </ul>
            </div>
          </div>
        </section>

        {/* Hardware Summary Section */}
        <section className={clsx(styles.section, styles.hardwareSummarySection, styles.altBackground)}>
          <div className="container">
            <h2 className={clsx("text--center", styles.sectionTitle)}>Hardware at a Glance</h2>
            <div className="row">
              <div className="col col--4">
                <div className={clsx("card shadow--md padding--lg", styles.hardwareCard)}>
                  <h3>Development Workstation</h3>
                  <ul>
                    <li>**OS**: Ubuntu 22.04 LTS</li>
                    <li>**CPU**: Modern Multi-core (i7/Ryzen 7+)</li>
                    <li>**RAM**: 32-64GB</li>
                    <li>**GPU**: NVIDIA RTX 4070 Ti+</li>
                  </ul>
                </div>
              </div>
              <div className="col col--4">
                <div className={clsx("card shadow--md padding--lg", styles.hardwareCard)}>
                  <h3>Edge Device (Robot Brain)</h3>
                  <ul>
                    <li>**Device**: NVIDIA Jetson Orin Nano/NX (8GB)</li>
                    <li>**OS**: JetPack 6.1 (Ubuntu 22.04)</li>
                    <li>**Sensors**: RealSense D455 (or similar), IMU</li>
                  </ul>
                </div>
              </div>
              <div className="col col--4">
                <div className={clsx("card shadow--md padding--lg", styles.hardwareCard)}>
                  <h3>Optional Physical Robot</h3>
                  <ul>
                    <li>**Model**: Unitree Go2 / OP3 / G1 (or similar)</li>
                    <li>**Integration**: ROS 2 Humble/Iron compatible</li>
                    <li>**Purpose**: Capstone project, real-world deployment</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className={clsx(styles.section, styles.ctaSection)}>
          <div className="container text--center">
            <h2 className={clsx("margin-bottom--lg", styles.ctaTitle)}>Ready to Build the Future of Robotics?</h2>
            <div className={styles.buttons}>
              <Link
                className={clsx("button button--secondary button--lg", styles.ctaButton)}
                to={useBaseUrl('/docs/preface')}>
                Start Your Journey Now!
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}