import React from 'react';
import { motion } from 'framer-motion';
import Link from '@docusaurus/Link';
import styles from './CtaSection.module.css';

export default function CtaSection() {
  return (
    <motion.div
      className={styles.ctaSection}
      initial={{ opacity: 0 }}
      whileInView={{ opacity: 1 }}
      viewport={{ once: true }}
      transition={{ duration: 1 }}
    >
      <div className={styles.ctaInner}>
        <h2 className={styles.ctaTitle}>Ready to get started?</h2>
        <p className={styles.ctaSubtitle}>
          Dive into the world of Physical AI and Humanoid Robotics today.
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/intro"
        >
          Start Learning
        </Link>
      </div>
    </motion.div>
  );
}