import React from 'react';
import { motion } from 'framer-motion';
import styles from './FeaturesSection.module.css';

const features = [
  {
    title: 'Comprehensive Content',
    description: 'From the basics of ROS 2 to advanced topics in humanoid robotics, our textbook covers it all.',
    icon: '📚',
  },
  {
    title: 'Interactive Examples',
    description: 'Learn by doing with our interactive examples and simulations.',
    icon: '🎮',
  },
  {
    title: 'AI-Powered Chatbot',
    description: 'Get instant help and answers to your questions with our integrated AI chatbot.',
    icon: '🤖',
  },
];

export default function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <motion.div
              key={idx}
              className="col col--4"
              initial={{ opacity: 0, y: 50 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.8, delay: idx * 0.2 }}
            >
              <div className={styles.feature}>
                <div className={styles.featureIcon}>{feature.icon}</div>
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}