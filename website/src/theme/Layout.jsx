import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatButton from '../components/Chatbot/FloatingChatButton';

// Custom layout wrapper that adds the chatbot to all pages
const Layout = (props) => {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatButton />
    </>
  );
};

export default Layout;