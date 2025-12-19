import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  // Remove SVG since we'll use different icons
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Cutting-Edge Research',
    description: ( 
      <>
        Explore the latest developments in Physical AI and Humanoid Robotics,
        including advanced perception systems, control algorithms, and learning methods.
      </>
    ),
  },
  {
    title: 'Practical Applications',
    description: (
      <>
        Discover real-world applications across industries including healthcare,
        manufacturing, service robotics, and human-robot collaboration.
      </>
    ),
  },
  {
    title: 'Ethical Considerations',
    description: (
      <>
        Understand the ethical implications and societal impact of advanced
        robotics, ensuring responsible development and deployment.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureIcon}>
          <div className={styles.iconCircle}>
            <span className={styles.iconText}>🤖</span>
          </div>
        </div>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
