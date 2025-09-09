#!/usr/bin/env python3
"""
GMINS GUI Main Application

Real-time monitoring and control interface for GMINS Navigation System.
Supports both AR1AFC and MAVLink protocols.

Usage:
    python main.py

Requirements:
    - PyQt5
    - pyserial  
    - numpy
    - matplotlib
    - pymavlink
"""

import sys
import os

# Add current directory to Python path for relative imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QIcon, QFont
except ImportError as e:
    print(f"❌ Error importing PyQt5: {e}")
    print("💡 Please install required packages:")
    print("   pip install -r requirements.txt")
    sys.exit(1)

from ui.main_window import MainWindow


class GMINSApplication(QApplication):
    """GMINS GUI Application Class"""
    
    def __init__(self, argv):
        super().__init__(argv)
        
        # Set application properties
        self.setApplicationName("GMINS Navigation System")
        self.setApplicationVersion("1.0.0")
        self.setOrganizationName("GMINS Team")
        
        # Set default font
        font = QFont("Arial", 10)
        self.setFont(font)
        
        # Set application style
        self.setStyle('Fusion')  # Modern cross-platform style
        
        # Create main window
        self.main_window = None
        self.init_main_window()
    
    def init_main_window(self):
        """Initialize the main window"""
        try:
            self.main_window = MainWindow()
            self.main_window.show()
            
            # Center the window on screen
            self.center_window()
            
        except Exception as e:
            QMessageBox.critical(
                None, 
                "Error", 
                f"Failed to initialize main window:\n{str(e)}"
            )
            sys.exit(1)
    
    def center_window(self):
        """Center the main window on screen"""
        if self.main_window:
            screen = self.desktop().screenGeometry()
            window = self.main_window.geometry()
            
            x = (screen.width() - window.width()) // 2
            y = (screen.height() - window.height()) // 2
            
            self.main_window.move(x, y)


def main():
    """Main entry point"""
    print("🚀 Starting GMINS GUI Application...")
    print("=" * 50)
    
    # Create application
    app = GMINSApplication(sys.argv)
    
    try:
        # Run application event loop
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\n👋 Application interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"❌ Application error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()