import React from 'react';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Home() {
  const aboutCards = [
    {
      title: 'Explore Cutting-Edge Robotics',
      description: 'Dive into Physical AI and Humanoid Robotics concepts, simulations, and applications.',
    },
    {
      title: 'Hands-on Learning',
      description: 'Understand AI-Robot brains, vision-language-action, and practical lab simulations.',
    },
    {
      title: 'Step-by-Step Modules',
      description: 'Follow structured modules from fundamentals to advanced AI integration in robotics.',
    },
  ];

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Comprehensive textbook on Physical AI & Humanoid Robotics"
    >
      {/* HERO SECTION */}
      <header className={styles.hero}>
        <div className={styles.heroContent}>
          <div className={styles.heroImage}>
            <img src="/img/humanoid-img.png" alt="Book Illustration" />
          </div>
          <div className={styles.heroText}>
            <h1 className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </h1>
            <p className={styles.heroSubtitle}>
              A hands-on journey through AI, simulation, and autonomous humanoid robotics.
            </p>
            <a href="/docs/module-1-ros-2/chapter-1-introduction-to-ros2" className={styles.primaryButton}>
              📖 Start Reading
            </a>
          </div>
          
        </div>
      </header>

      {/* ABOUT THE BOOK */}
      <section className={styles.aboutSection}>
        <h2 className={styles.sectionTitle}>What You’ll Learn</h2>
        <div className={styles.cardGrid}>
          {aboutCards.map((card, idx) => (
            <div key={idx} className={styles.card}>
              <h3>{card.title}</h3>
              <p>{card.description}</p>
            </div>
          ))}
        </div>
      </section>
    </Layout>
  );
}
