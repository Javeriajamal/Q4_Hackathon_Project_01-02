import type {ReactNode, FormEvent} from 'react';
import { useState } from 'react';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import clsx from 'clsx';
import styles from './contact.module.css';

type FormData = {
  email: string;
  message: string;
};

type FormStatus = {
  type: 'idle' | 'loading' | 'success' | 'error';
  message: string;
};

function WaitlistForm() {
  const [formData, setFormData] = useState<FormData>({ email: '', message: '' });
  const [formStatus, setFormStatus] = useState<FormStatus>({ type: 'idle', message: '' });

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = (e: FormEvent) => {
    e.preventDefault();
    setFormStatus({ type: 'loading', message: 'Submitting...' });

    // Validate email format
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(formData.email)) {
      setFormStatus({ type: 'error', message: 'Please enter a valid email address.' });
      return;
    }

    try {
      // Save email to localStorage
      const existingEmails = JSON.parse(localStorage.getItem('waitlistEmails') || '[]');
      if (!existingEmails.includes(formData.email)) {
        existingEmails.push(formData.email);
        localStorage.setItem('waitlistEmails', JSON.stringify(existingEmails));
      }

      // Save the current submission
      localStorage.setItem('lastWaitlistSubmission', formData.email);

      setFormStatus({
        type: 'success',
        message: 'Thank you! Your email has been saved to our waitlist. We will notify you when the book is available.'
      });

      // Reset form
      setFormData({ email: '', message: '' });
    } catch (error) {
      setFormStatus({ type: 'error', message: 'An error occurred. Please try again.' });
    }
  };

  return (
    <div className={styles.contactForm}>
      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.formGroup}>
          <label htmlFor="email" className={styles.label}>
            Email Address <span className={styles.required}>*</span>
          </label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            className={clsx('form-control', styles.input)}
            placeholder="your.email@example.com"
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="message" className={styles.label}>
            Message (Optional)
          </label>
          <textarea
            id="message"
            name="message"
            value={formData.message}
            onChange={handleChange}
            rows={4}
            className={clsx('form-control', styles.textarea)}
            placeholder="Let us know what you're most excited about in Physical AI and Humanoid Robotics..."
          />
        </div>

        <button
          type="submit"
          className={clsx('button button--primary', styles.submitButton)}
          disabled={formStatus.type === 'loading'}
        >
          {formStatus.type === 'loading' ? 'Submitting...' : 'Join Waitlist'}
        </button>

        {formStatus.type !== 'idle' && (
          <div className={clsx(
            'alert',
            formStatus.type === 'success' ? 'alert--success' :
            formStatus.type === 'error' ? 'alert--danger' : 'alert--secondary',
            styles.statusMessage
          )}>
            {formStatus.message}
          </div>
        )}
      </form>

      <div className={styles.privacyNotice}>
        <p><strong>Privacy Notice:</strong> Your email will only be stored in your browser's localStorage and will not be transmitted to any server. This ensures your privacy while allowing you to join our waitlist.</p>
      </div>
    </div>
  );
}

export default function Contact(): ReactNode {
  return (
    <Layout title="Contact" description="Join our waitlist for updates on the Physical AI and Humanoid Robotics book">
      <main className={clsx('container', styles.contactPage)}>
        <div className="hero text--center">
          <div className="container padding-horiz--md">
            <Heading as="h1" className="hero__title">
              Contact & Updates
            </Heading>
            <p className="hero__subtitle">
              Join our waitlist to receive updates when the book is available
            </p>
          </div>
        </div>

        <div className="row margin-vert--lg">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__body">
                <WaitlistForm />
              </div>
            </div>
          </div>
        </div>

        <div className="row margin-vert--lg">
          <div className="col col--8 col--offset-2">
            <div className={styles.informationSection}>
              <Heading as="h2">About the Book</Heading>
              <p>
                "Physical AI and Humanoid Robotics" provides a comprehensive guide to the intersection of artificial intelligence
                and physical systems. The book covers fundamental principles, current applications, and future directions in
                humanoid robotics.
              </p>
              <p>
                Written for researchers, engineers, and students, this book explores both the technical aspects and ethical
                implications of Physical AI systems that interact with the physical world.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}