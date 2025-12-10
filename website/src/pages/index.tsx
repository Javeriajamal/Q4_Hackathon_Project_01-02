import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import styles from './index.module.css';
//import HeroImage from '@site/static/img/robotics-img.png';


function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>

      <img src="/img/robotics-img.png" alt="hero image" 
        style={{ 
        width:'370px' ,
        height:'370px',
        marginLeft: '130px',
        borderRadius: '15px'
       }} 
 />
      

      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            Physical AI and Humanoid Robotics
          </Heading>
          <p className="hero__subtitle">
            A comprehensive guide to the intersection of artificial intelligence and physical systems
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/book">
              Start Reading - 20min ⏱️
            </Link>
            <Link
              className="button button--outline button--primary button--lg margin-left--md"
              to="/about">
              Learn More
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function BookPreview() {
  return (
    <section className={clsx(styles.bookPreview, 'margin-vert--md padding-vert--lg')}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center">
            <Heading as="h2">About This Book</Heading>
            <p className="padding-horiz--md">
              This comprehensive guide explores the cutting-edge field of Physical AI and Humanoid Robotics,
              where artificial intelligence meets the physical world. From fundamental principles to advanced
              applications, this book provides both technical depth and practical insights.
            </p>
          </div>
        </div>

        <div className="row padding-vert--lg">
          <div className="col col--4">
            <div className="card">
              <div className="card__header text--center">
                <h3>Physical AI Foundations</h3>
              </div>
              <div className="card__body">
                <p>
                  Understand how AI systems interact with the physical world through sensors, actuators,
                  and real-time decision making.
                </p>
              </div>
            </div>
          </div>

          <div className="col col--4">
            <div className="card">
              <div className="card__header text--center">
                <h3>Humanoid Robotics</h3>
              </div>
              <div className="card__body">
                <p>
                  Explore the mechanics, control systems, and design principles behind human-like robots.
                </p>
              </div>
            </div>
          </div>

          <div className="col col--4">
            <div className="card">
              <div className="card__header text--center">
                <h3>Applications & Ethics</h3>
              </div>
              <div className="card__body">
                <p>
                  Discover real-world applications and consider the ethical implications of advanced robotics.
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI and Humanoid Robotics Book`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <BookPreview />
      </main>
    </Layout>
  );
}
