import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatbotWidget.module.css';
import ChatMessage from './ChatMessage';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  is_loading?: boolean;
}

const ChatbotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null); // Ref for input focus

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
    if (isOpen) {
      inputRef.current?.focus(); // Focus input when chatbot opens
    }
  }, [messages, isOpen]);

  const handleSendMessage = async () => {
    if (inputValue.trim() === '') return;

    const newUserMessage: Message = {
      id: Date.now().toString() + '-user',
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages((prevMessages) => [...prevMessages, newUserMessage]);
    setInputValue('');
    setIsLoading(true);

    // Simulate bot typing
    const botLoadingMessage: Message = {
      id: Date.now().toString() + '-bot-loading',
      text: '...',
      sender: 'bot',
      timestamp: new Date(),
      is_loading: true,
    };
    setMessages((prevMessages) => [...prevMessages, botLoadingMessage]);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: inputValue }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botResponseText = data.response || data.error || 'An unexpected error occurred.';

      setMessages((prevMessages) =>
        prevMessages.map((msg) =>
          msg.id === botLoadingMessage.id ? { ...msg, text: botResponseText, is_loading: false } : msg
        )
      );
    } catch (error) {
      console.error('Error communicating with FastAPI backend:', error);
      setMessages((prevMessages) =>
        prevMessages.map((msg) =>
          msg.id === botLoadingMessage.id ? { ...msg, text: 'Error: Could not get a response.', is_loading: false } : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !isLoading) {
      handleSendMessage();
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <button 
        className={styles.toggleButton} 
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chatbot' : 'Open chatbot'}
        aria-expanded={isOpen}
      >
        {isOpen ? '✖' : '💬'}
      </button>
      {isOpen && (
        <div className={styles.chatbotWindow} role="dialog" aria-labelledby="chatbot-title">
          <h2 id="chatbot-title" className="sr-only">Chatbot</h2> {/*sr-only for screen readers*/}
          <div className={styles.messagesContainer} role="log" aria-live="polite">
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                Hi there! Ask me anything about Physical AI and Humanoid Robotics.
              </div>
            )}
            {messages.map((msg) => (
              <ChatMessage key={msg.id} message={msg} />
            ))}
            <div ref={messagesEndRef} />
          </div>
          <div className={styles.inputContainer}>
            <input
              ref={inputRef} // Attach ref
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              disabled={isLoading}
              aria-label="Chat input"
            />
            <button onClick={handleSendMessage} disabled={isLoading} aria-label="Send message">
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;