import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import clsx from 'clsx';
import styles from './book.module.css';

function BookNavigation() {
  return (
    <nav className={clsx('navbar', styles.bookNavbar)}>
      <div className="container">
        <div className="navbar__inner">
          <div className="navbar__items">
            <Link className="navbar__brand" to="/">
              <span className="navbar__title">Physical AI & Robotics</span>
            </Link>
          </div>
          <div className="navbar__items navbar__items--right">
            <Link className="navbar__item navbar__link" to="/docs/book">
              Table of Contents
            </Link>
            <Link className="button button--primary" to="/contact">
              Updates
            </Link>
          </div>
        </div>
      </div>
    </nav>
  );
}

function BookIntro() {
  return (
    <section className={styles.bookIntro}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center padding-vert--lg">
            <Heading as="h1">Physical AI and Humanoid Robotics</Heading>
            <p className="hero__subtitle">
              A comprehensive guide to the intersection of artificial intelligence and physical systems
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ChapterPreview() {
  return (
    <section className={styles.chapterPreview}>
      <div className="container">
        <div className="row">
          <div className="col col--12 padding-vert--lg">
            <Heading as="h2" className="text--center margin-bottom--lg">Book Structure</Heading>
          </div>
        </div>

        <div className="row">
          <div className="col col--4">
            <div className="card">
              <div className="card__header">
                <h3>Chapter 1: Introduction to Physical AI</h3>
              </div>
              <div className="card__body">
                <p>Foundations and principles of Physical AI, current state and applications</p>
              </div>
              <div className="card__footer">
                <Link to="/docs/book/chapter-1/foundations-and-principles" className="button button--primary button--block">
                  Read Chapter
                </Link>
              </div>
            </div>
          </div>

          <div className="col col--4">
            <div className="card">
              <div className="card__header">
                <h3>Chapter 2: Humanoid Robotics Fundamentals</h3>
              </div>
              <div className="card__body">
                <p>Design and mechanics, control systems and actuation</p>
              </div>
              <div className="card__footer">
                <Link to="/docs/book/chapter-2/design-and-mechanics" className="button button--primary button--block">
                  Read Chapter
                </Link>
              </div>
            </div>
          </div>

          <div className="col col--4">
            <div className="card">
              <div className="card__header">
                <h3>Chapter 3: AI Integration in Robotics</h3>
              </div>
              <div className="card__body">
                <p>Perception and sensing, decision making and learning</p>
              </div>
              <div className="card__footer">
                <Link to="/docs/book/chapter-3/perception-and-sensing" className="button button--primary button--block">
                  Read Chapter
                </Link>
              </div>
            </div>
          </div>
        </div>

        <div className="row margin-top--lg">
          <div className="col col--4 col--offset-2">
            <div className="card">
              <div className="card__header">
                <h3>Chapter 4: Applications and Use Cases</h3>
              </div>
              <div className="card__body">
                <p>Industrial and service robotics, healthcare and social robotics</p>
              </div>
              <div className="card__footer">
                <Link to="/docs/book/chapter-4/industrial-and-service-robotics" className="button button--primary button--block">
                  Read Chapter
                </Link>
              </div>
            </div>
          </div>

          <div className="col col--4">
            <div className="card">
              <div className="card__header">
                <h3>Chapter 5: Future Directions and Ethics</h3>
              </div>
              <div className="card__body">
                <p>Emerging technologies and trends, ethical considerations</p>
              </div>
              <div className="card__footer">
                <Link to="/docs/book/chapter-5/emerging-technologies-and-trends" className="button button--primary button--block">
                  Read Chapter
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ReadingProgress() {
  return (
    <section className={styles.progressSection}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className={styles.progressBarContainer}>
              <div className={styles.progressBar}>
                <div className={styles.progressFill} style={{ width: '20%' }}></div>
              </div>
              <div className="margin-top--sm text--center">
                <p>You've read 1 of 5 chapters (20%)</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Book(): ReactNode {
  return (
    <Layout title="Physical AI and Humanoid Robotics Book" description="Complete book on Physical AI and Humanoid Robotics">
      <BookNavigation />
      <main className={styles.bookPage}>
        <BookIntro />
        <ReadingProgress />
        <ChapterPreview />
      </main>
    </Layout>
  );
}