# 🤖 RoboControl - ESP32 Robot Control Panel

<div align="center">

![RoboControl](https://img.shields.io/badge/RoboControl-v2.0-00f0ff?style=for-the-badge&labelColor=0a0a0f)
![ESP32](https://img.shields.io/badge/ESP32-Compatible-00ff88?style=for-the-badge&labelColor=0a0a0f)
![License](https://img.shields.io/badge/License-MIT-bf00ff?style=for-the-badge&labelColor=0a0a0f)

**A sleek, cyberpunk-themed web interface for controlling robotic arms and mobile robots via ESP32**

*Developed by DigiClub*

</div>

---

## ✨ Features

### 🎮 Motor Control
- **Directional Movement**: Forward, backward, left, and right controls
- **Touch & Click Support**: Works seamlessly on both desktop and mobile devices
- **Stop Button**: Emergency stop functionality for immediate motor halt

### ⚙️ Servo Control
- **3-Axis Control**: Manage up to 3 servo motors simultaneously
  - **Servo 01**: Base Rotation (0° - 180°)
  - **Servo 02**: Shoulder Joint (0° - 180°)
  - **Servo 03**: Gripper Claw (0° - 180°)
- **Real-time Feedback**: Live angle display for each servo

### 💾 Trajectory Memory System
- **Record Mode**: Capture movement sequences
- **Playback**: Execute recorded trajectories automatically
- **Clear Function**: Reset memory for new recordings
- **Step-by-step Execution**: Visual feedback during playback

### 🎨 Design & UX
- **Cyberpunk Aesthetic**: Futuristic neon-glow design
- **Animated Background**: Circuit board patterns and floating particles
- **Responsive Layout**: Optimized for all screen sizes
- **PWA Support**: Install as a standalone app on mobile devices
- **Dark Theme**: Easy on the eyes during extended use

---

## 🚀 Getting Started

### Prerequisites

- ESP32 microcontroller
- Servo motors (up to 3)
- DC motors with motor driver (for movement)
- Wi-Fi network

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/Mehdirben/robotic.git
   cd robotic
   ```

2. **Upload to ESP32**
   
   The `index.html` file should be served by the ESP32's web server. You can either:
   - Embed it directly in your ESP32 code using PROGMEM
   - Serve it from SPIFFS/LittleFS filesystem
   - Host it externally and configure CORS on your ESP32

3. **Configure ESP32 Endpoints**
   
   Your ESP32 should handle these HTTP endpoints:

   | Endpoint | Description |
   |----------|-------------|
   | `/forward` | Move robot forward |
   | `/backward` | Move robot backward |
   | `/left` | Rotate robot left |
   | `/right` | Rotate robot right |
   | `/stop` | Stop all motors |
   | `/set?servo=X&angle=Y` | Set servo X to angle Y |

### Usage

1. Connect to the same Wi-Fi network as your ESP32
2. Open the control panel in your browser (ESP32's IP address)
3. Use the directional buttons for movement (hold to move, release to stop)
4. Adjust servo sliders to control the robotic arm
5. Use the memory system to record and replay movement sequences

---

## 📱 Mobile Installation (PWA)

RoboControl can be installed as a Progressive Web App:

1. Open the control panel in Chrome/Safari
2. Tap the browser menu (⋮ or share icon)
3. Select "Add to Home Screen" or "Install App"
4. Launch from your home screen for a native app experience

---

## 🔧 API Reference

### Motor Control Commands

```javascript
// Movement functions
forward()   // Start moving forward
backward()  // Start moving backward
left()      // Rotate left
right()     // Rotate right
stopMotors() // Stop all motors
```

### Servo Control

```javascript
// Set servo position
setServo(servoNumber, angle)
// Example: setServo(1, 90) - Set servo 1 to 90 degrees
```

### Memory System

```javascript
// Recording actions are handled via button clicks:
// - Record: Start recording movements
// - Confirm: Stop recording
// - Execute: Play back recorded sequence
// - Clear: Reset memory
```

---

## 🎨 Customization

### Color Scheme

The interface uses CSS custom properties for easy theming:

```css
:root {
  --cyber-blue: #00f0ff;    /* Primary accent */
  --cyber-purple: #bf00ff;  /* Secondary accent */
  --cyber-pink: #ff00aa;    /* Highlight */
  --cyber-green: #00ff88;   /* Success states */
  --cyber-red: #ff0055;     /* Danger/Stop */
  --bg-dark: #0a0a0f;       /* Background */
}
```

### Adding More Servos

To add additional servos, duplicate a servo group in the HTML and update the JavaScript accordingly.

---

## 📁 Project Structure

```
robotic/
├── index.html    # Complete web interface (single-file application)
└── README.md     # Documentation
```

---

## 🤝 Contributing

Contributions are welcome! Feel free to:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

---

## 🙏 Acknowledgments

- **DigiClub** - Project development and design
- ESP32 community for inspiration and support
- Font families: Orbitron, Rajdhani, Share Tech Mono

---

<div align="center">

**⚡ Powered by DigiClub | ESP32 Neural Control System v2.0 ⚡**

</div>
