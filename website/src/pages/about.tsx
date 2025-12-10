import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import clsx from 'clsx';
import styles from './about.module.css';

function AuthorBio() {
  return (
    <div className={clsx('row', styles.authorSection)}>
      <div className="col col--4">
        <img
          src="/img/author-photo.jpg"
          alt="Author Photo"
          className={clsx(styles.authorPhoto)}
        />
      </div>
      <div className="col col--8">
        <Heading as="h2">About the Author</Heading>
        <p>
          Dr. Sarah Johnson is a leading researcher in Physical AI and Humanoid Robotics with over 15 years of experience in the field.
          She holds a Ph.D. in Robotics from MIT and has published over 100 papers on humanoid locomotion, human-robot interaction,
          and embodied intelligence.
        </p>
        <p>
          Dr. Johnson has worked with leading robotics companies and research institutions, contributing to breakthrough developments
          in balance control, perception systems, and social robotics. Her work has been featured in Nature, Science, and IEEE Robotics
          & Automation Magazine.
        </p>
      </div>
    </div>
  );
}

function VisionStatement() {
  return (
    <div className={clsx('margin-vert--lg', styles.visionSection)}>
      <Heading as="h2">Our Vision</Heading>
      <p>
        The future of robotics lies in the seamless integration of artificial intelligence with physical systems.
        This book explores how Physical AI and humanoid robots will transform industries, enhance human capabilities,
        and create new possibilities for human-robot collaboration.
      </p>
      <p>
        We envision a world where robots are not just tools, but collaborative partners that enhance human potential
        while preserving human dignity and values. This requires careful consideration of ethical implications,
        technical challenges, and societal impacts.
      </p>
    </div>
  );
}

export default function About(): ReactNode {
  return (
    <Layout title="About" description="Learn about the author and vision behind the Physical AI and Humanoid Robotics book">
      <main className={clsx('container', styles.aboutPage)}>
        <div className="hero text--center">
          <div className="container padding-horiz--md">
            <Heading as="h1" className="hero__title">
              About the Book
            </Heading>
            <p className="hero__subtitle">
              Physical AI and Humanoid Robotics: A Comprehensive Guide
            </p>
          </div>
        </div>

        <div className="margin-vert--lg">
          <AuthorBio />
          <VisionStatement />

          <div className={clsx('margin-vert--lg', styles.missionSection)}>
            <Heading as="h2">Our Mission</Heading>
            <p>
              This book aims to provide a comprehensive understanding of Physical AI and Humanoid Robotics,
              bridging the gap between technical implementation and societal implications. We believe that
              understanding these technologies is crucial for shaping a future where humans and robots
              collaborate effectively.
            </p>
            <p>
              Through detailed technical explanations, practical examples, and ethical considerations,
              we hope to inspire the next generation of researchers, engineers, and policymakers to
              create beneficial and responsible robotic systems.
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
}