import React from 'react';
import { motion } from 'framer-motion';
import styles from './ModulesSection.module.css';

const modules = [
  {
    title: 'Module 1: Introduction to ROS 2',
    description: 'Learn the fundamentals of the Robot Operating System 2.',
    link: '/docs/module-1/introduction',
  },
  {
    title: 'Module 2: Simulation with Gazebo',
    description: 'Simulate your robots in a virtual environment.',
    link: '/docs/module-2/introduction',
  },
  {
    title: 'Module 3: NVIDIA Isaac AI Stack',
    description: 'Explore advanced AI perception, simulation, and deployment.',
    link: '/docs/module-3',
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description: 'Explore how vision and language models drive robotic actions.',
    link: '/docs/module-4/ros_vla_integration',
  },
];

export default function ModulesSection() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Modules</h2>
        <div className="row">
          {modules.map((module, idx) => (
            <motion.div
              key={idx}
              className="col col--6"
              initial={{ opacity: 0, y: 50 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.8, delay: idx * 0.2 }}
            >
              <a href={module.link} className={styles.moduleLink}>
                <div className={styles.module}>
                  <h3>{module.title}</h3>
                  <p>{module.description}</p>
                </div>
              </a>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}