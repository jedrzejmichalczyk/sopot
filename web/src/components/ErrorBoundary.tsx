import React, { Component, ErrorInfo, ReactNode } from 'react';

interface Props {
  children: ReactNode;
  fallback?: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
}

/**
 * ErrorBoundary: Catches React errors and displays a fallback UI
 *
 * Usage:
 * <ErrorBoundary fallback={<CustomFallback />}>
 *   <YourComponent />
 * </ErrorBoundary>
 */
class ErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
      errorInfo: null,
    };
  }

  static getDerivedStateFromError(error: Error): Partial<State> {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    console.error('ErrorBoundary caught an error:', error, errorInfo);
    this.setState({ error, errorInfo });
  }

  handleReset = (): void => {
    this.setState({ hasError: false, error: null, errorInfo: null });
    window.location.reload();
  };

  render(): ReactNode {
    if (this.state.hasError) {
      // Use custom fallback if provided
      if (this.props.fallback) {
        return this.props.fallback;
      }

      // Default fallback UI
      return (
        <div style={{
          display: 'flex',
          flexDirection: 'column',
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
            <h1 style={{
              fontSize: '1.5rem',
              marginBottom: '1rem',
              color: '#ef4444',
            }}>
              ⚠️ Something went wrong
            </h1>

            <p style={{
              marginBottom: '1rem',
              color: '#aaa',
            }}>
              The application encountered an unexpected error.
            </p>

            {this.state.error && (
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
                  margin: '0.5rem 0',
                }}>
                  {this.state.error.toString()}
                </pre>
                {this.state.errorInfo && (
                  <pre style={{
                    fontSize: '0.75rem',
                    overflow: 'auto',
                    color: '#888',
                    marginTop: '0.5rem',
                  }}>
                    {this.state.errorInfo.componentStack}
                  </pre>
                )}
              </details>
            )}

            <button
              onClick={this.handleReset}
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
              Reload Application
            </button>

            <div style={{
              marginTop: '1.5rem',
              padding: '1rem',
              backgroundColor: '#1a1a1a',
              borderRadius: '4px',
              fontSize: '0.875rem',
              color: '#888',
            }}>
              <p style={{ margin: '0 0 0.5rem 0', fontWeight: 'bold' }}>
                Troubleshooting:
              </p>
              <ul style={{ margin: 0, paddingLeft: '1.5rem' }}>
                <li>Check your browser console for more details</li>
                <li>Try clearing your browser cache and reloading</li>
                <li>Ensure you're using a modern browser with WebAssembly support</li>
                <li>Report persistent issues on GitHub</li>
              </ul>
            </div>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;
