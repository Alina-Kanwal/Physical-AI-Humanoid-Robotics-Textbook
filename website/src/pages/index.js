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
            Read the Book 📚
          </Link>
        </div>
      </div>
    </header>
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
        <div className="container">
          <div className="row">
            <div className="col col--6 col--offset-3 padding-horiz--md">
              <h2>Physical AI & Humanoid Robotics</h2>
              <p>
                This comprehensive guide covers Physical AI using ROS 2, Gazebo, Unity, and NVIDIA Isaac.
                Explore cutting-edge techniques for developing intelligent robotic systems.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}