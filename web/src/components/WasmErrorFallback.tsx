import React from 'react';

interface WasmErrorFallbackProps {
  error?: Error | string;
}

/**
 * WasmErrorFallback: Specialized error UI for WebAssembly loading failures
 */
const WasmErrorFallback: React.FC<WasmErrorFallbackProps> = ({ error }) => {
  const errorMessage = typeof error === 'string' ? error : error?.message || 'Unknown error';

  const handleReload = () => {
    window.location.reload();
  };

  // Check if it's a loading error
  const isLoadingError = errorMessage.toLowerCase().includes('load') ||
                         errorMessage.toLowerCase().includes('fetch') ||
                         errorMessage.toLowerCase().includes('module');

  return (
    <div style={{
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      minHeight: '100vh',
      padding: '2rem',
      backgroundColor: '#1a1a1a',
      color: '#fff',
      fontFamily: 'system-ui, -apple-system, sans-serif',
    }}>
      <div style={{
        maxWidth: '600px',
        width: '100%',
        backgroundColor: '#2a2a2a',
        borderRadius: '8px',
        padding: '2rem',
        boxShadow: '0 4px 6px rgba(0, 0, 0, 0.3)',
      }}>
        <div style={{
          fontSize: '3rem',
          textAlign: 'center',
          marginBottom: '1rem',
        }}>
          🚀
        </div>

        <h1 style={{
          fontSize: '1.5rem',
          marginBottom: '1rem',
          textAlign: 'center',
          color: '#ef4444',
        }}>
          Failed to Load Simulation Engine
        </h1>

        <p style={{
          marginBottom: '1rem',
          color: '#aaa',
          textAlign: 'center',
        }}>
          The WebAssembly simulation module could not be loaded.
        </p>

        <details style={{
          marginBottom: '1.5rem',
          backgroundColor: '#1a1a1a',
          padding: '1rem',
          borderRadius: '4px',
          border: '1px solid #444',
        }}>
          <summary style={{
            cursor: 'pointer',
            fontWeight: 'bold',
            marginBottom: '0.5rem',
          }}>
            Error Details
          </summary>
          <pre style={{
            fontSize: '0.875rem',
            overflow: 'auto',
            color: '#ef4444',
            whiteSpace: 'pre-wrap',
            wordBreak: 'break-word',
          }}>
            {errorMessage}
          </pre>
        </details>

        <div style={{
          backgroundColor: '#1a1a1a',
          padding: '1rem',
          borderRadius: '4px',
          marginBottom: '1.5rem',
          fontSize: '0.875rem',
          color: '#888',
        }}>
          <p style={{ margin: '0 0 0.5rem 0', fontWeight: 'bold' }}>
            Common Solutions:
          </p>
          <ul style={{ margin: 0, paddingLeft: '1.5rem' }}>
            {isLoadingError ? (
              <>
                <li>Check your internet connection</li>
                <li>Try reloading the page (WASM module may still be downloading)</li>
                <li>Disable browser extensions that might block WebAssembly</li>
              </>
            ) : (
              <>
                <li>Ensure your browser supports WebAssembly</li>
                <li>Update your browser to the latest version</li>
                <li>Try a different browser (Chrome, Firefox, Safari, Edge)</li>
              </>
            )}
            <li>Clear your browser cache and reload</li>
            <li>Check the browser console (F12) for more details</li>
          </ul>
        </div>

        <div style={{
          display: 'flex',
          gap: '1rem',
          justifyContent: 'center',
        }}>
          <button
            onClick={handleReload}
            style={{
              backgroundColor: '#3b82f6',
              color: '#fff',
              padding: '0.75rem 1.5rem',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer',
              fontWeight: 'bold',
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.backgroundColor = '#2563eb';
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.backgroundColor = '#3b82f6';
            }}
          >
            Reload Page
          </button>

          <a
            href="https://github.com/anthropics/sopot/issues"
            target="_blank"
            rel="noopener noreferrer"
            style={{
              backgroundColor: '#444',
              color: '#fff',
              padding: '0.75rem 1.5rem',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer',
              fontWeight: 'bold',
              textDecoration: 'none',
              display: 'inline-block',
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.backgroundColor = '#555';
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.backgroundColor = '#444';
            }}
          >
            Report Issue
          </a>
        </div>

        <div style={{
          marginTop: '1.5rem',
          textAlign: 'center',
          fontSize: '0.75rem',
          color: '#666',
        }}>
          <p>
            SOPOT requires WebAssembly support.{' '}
            <a
              href="https://webassembly.org/"
              target="_blank"
              rel="noopener noreferrer"
              style={{ color: '#3b82f6' }}
            >
              Learn more about WebAssembly
            </a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default WasmErrorFallback;
