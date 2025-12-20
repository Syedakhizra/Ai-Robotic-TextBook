import React from 'react';
import { motion } from 'framer-motion';
import Link from '@docusaurus/Link';
import styles from './HeroSection.module.css';

export default function HeroSection() {
  return (
    <motion.div
      className={styles.hero}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 1 }}
    >
      <div className={styles.heroInner}>
        <motion.h1
          className={styles.heroTitle}
          initial={{ y: -50, opacity: 0 }}
          animate={{ y: 0, opacity: 1 }}
          transition={{ duration: 0.8, delay: 0.5 }}
        >
          Physical AI & Humanoid Robotics Textbook
        </motion.h1>
        <motion.p
          className={styles.heroSubtitle}
          initial={{ y: 50, opacity: 0 }}
          animate={{ y: 0, opacity: 1 }}
          transition={{ duration: 0.8, delay: 0.8 }}
        >
          A complete AI-native textbook + Docusaurus site for Physical AI & Humanoid Robotics
        </motion.p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/preface"
          >
            Start Learning
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs"
          >
            View Modules
          </Link>
        </div>
      </div>
    </motion.div>
  );
}