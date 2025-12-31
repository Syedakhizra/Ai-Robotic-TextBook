import React from 'react';
import styles from './ChatMessage.module.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  is_loading?: boolean;
}

interface ChatMessageProps {
  message: Message;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const messageClass = message.sender === 'user' ? styles.userMessage : styles.botMessage;
  const timestamp = new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  return (
    <div className={`${styles.messageContainer} ${messageClass}`}>
      <div className={styles.messageBubble}>
        {message.is_loading ? (
          <div className={styles.loadingDots}>
            <span>.</span><span>.</span><span>.</span>
          </div>
        ) : (
          message.text
        )}
      </div>
      <div className={styles.messageInfo}>
        <span className={styles.sender}>{message.sender === 'user' ? 'You' : 'Bot'}</span>
        <span className={styles.timestamp}>{timestamp}</span>
      </div>
    </div>
  );
};

export default ChatMessage;