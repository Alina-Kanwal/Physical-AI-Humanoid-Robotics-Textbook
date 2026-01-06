import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading 📚
          </Link>
          <Link
            className="button button--primary button--lg margin-left--md"
            to="/docs/module-0-fundamentals/introduction-to-physical-ai">
            Explore Physical AI →
          </Link>
        </div>
      </div>
    </header>
  );
}

function Features() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>Physical AI Fundamentals</h3>
              <p>Understand the principles of embodied intelligence, sensorimotor integration, and real-world AI systems.</p>
              <Link to="/docs/module-0-fundamentals/introduction-to-physical-ai">
                Learn More →
              </Link>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>Humanoid Robotics</h3>
              <p>Explore the design principles, control systems, and cognitive architectures for human-like robots.</p>
              <Link to="/docs/module-0-fundamentals/humanoid-robot-design">
                Learn More →
              </Link>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>AI Brain Systems</h3>
              <p>Discover cognitive architectures, embodied learning, and neural approaches to robot intelligence.</p>
              <Link to="/docs/module-5-ai-brain-humanoid/cognitive-architectures">
                Learn More →
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesOverview() {
  return (
    <section className={clsx('margin-vert--lg', styles.modules)}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center">
            <h2>Complete Learning Path</h2>
            <p className="padding-horiz--md">
              From ROS 2 fundamentals to advanced AI brain systems, build comprehensive humanoid robots
            </p>
          </div>
        </div>

        <div className="row padding-vert--md">
          <div className="col col--3">
            <div className="card">
              <div className="card__header">
                <h3>Module 0: Fundamentals</h3>
              </div>
              <div className="card__body">
                <p>Physical AI principles, humanoid design, physics simulation, sensors & actuators</p>
              </div>
              <div className="card__footer">
                <Link
                  className="button button--primary button--block"
                  to="/docs/module-0-fundamentals/introduction-to-physical-ai">
                  Explore →
                </Link>
              </div>
            </div>
          </div>

          <div className="col col--3">
            <div className="card">
              <div className="card__header">
                <h3>Module 1: ROS 2</h3>
              </div>
              <div className="card__body">
                <p>Robotic operating system, communication, control, and node management</p>
              </div>
              <div className="card__footer">
                <Link
                  className="button button--primary button--block"
                  to="/docs/module-1-ros2/introduction-to-ros2">
                  Explore →
                </Link>
              </div>
            </div>
          </div>

          <div className="col col--3">
            <div className="card">
              <div className="card__header">
                <h3>Module 5: AI Brain</h3>
              </div>
              <div className="card__body">
                <p>Cognitive architectures, embodied AI, perception-action loops</p>
              </div>
              <div className="card__footer">
                <Link
                  className="button button--primary button--block"
                  to="/docs/module-5-ai-brain-humanoid/cognitive-architectures">
                  Explore →
                </Link>
              </div>
            </div>
          </div>

          <div className="col col--3">
            <div className="card">
              <div className="card__header">
                <h3>Capstone Project</h3>
              </div>
              <div className="card__body">
                <p>Build an autonomous humanoid robot integrating all concepts</p>
              </div>
              <div className="card__footer">
                <Link
                  className="button button--primary button--block"
                  to="/docs/capstone-project/index">
                  Start Building →
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <Features />
        <ModulesOverview />
        <section className="margin-vert--lg">
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2 text--center">
                <h2>Ready to Build Intelligent Humanoid Robots?</h2>
                <p>
                  Join thousands of researchers and engineers mastering Physical AI and humanoid robotics.
                  Start your journey today with this comprehensive guide.
                </p>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/intro">
                  Begin Your Journey
                </Link>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}