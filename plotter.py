#!/usr/bin/env python3
"""
PID Robot Controller GUI
Reads TLV data from ESP32, plots angle data, and sends PID parameters
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np

class TLVParser:
    """TLV Parser for Python to match ESP32 format"""
    
    # TLV Types matching ESP32 code exactly
    TLV_TYPE_SET_TARGET_ANGLE = 0x01
    TLV_TYPE_SET_PRINCIPLE    = 0x02
    TLV_TYPE_SET_INTEGRAL     = 0x03
    TLV_TYPE_SET_DERIVITIVE   = 0x04
    TLV_TYPE_SET_DEADBAND     = 0x05
    TLV_TYPE_SET_BASESPEED    = 0x06
    TLV_TYPE_GET_TARGET_ANGLE = 0x07
    TLV_TYPE_GET_PRINCIPLE    = 0x08
    TLV_TYPE_GET_INTEGRAL     = 0x09
    TLV_TYPE_GET_DERIVITIVE   = 0x0A
    TLV_TYPE_GET_DEADBAND     = 0x0B
    TLV_TYPE_GET_BASESPEED    = 0x0C
    TLV_TYPE_CURRENT_ANGLE    = 0x0D
    
    def __init__(self):
        self.buffer = bytearray()
        
    def parse_tlv_data(self, data):
        """Parse TLV data and return list of TLV entries"""
        tlv_entries = []
        pos = 0
        
        while pos < len(data):
            # Need at least 3 bytes for Type + Length
            if pos + 3 > len(data):
                break
                
            # Parse Type (1 byte)
            tlv_type = data[pos]
            pos += 1
            
            # Parse Length (2 bytes, big-endian)
            length = struct.unpack('>H', data[pos:pos+2])[0]
            pos += 2
            
            # Check if we have enough data for Value
            if pos + length > len(data):
                break
                
            # Extract Value
            value = data[pos:pos+length]
            pos += length
            
            tlv_entries.append({
                'type': tlv_type,
                'length': length,
                'value': value
            })
            
        return tlv_entries
    
    def get_float_value(self, tlv_entry, big_endian=True):
        """Convert TLV value to float"""
        if len(tlv_entry['value']) != 4:
            return None
            
        if big_endian:
            return struct.unpack('>f', tlv_entry['value'])[0]
        else:
            return struct.unpack('<f', tlv_entry['value'])[0]
    
    def get_int_value(self, tlv_entry, big_endian=True):
        """Convert TLV value to integer"""
        value = tlv_entry['value']
        if len(value) == 1:
            return struct.unpack('B', value)[0]
        elif len(value) == 2:
            return struct.unpack('>H' if big_endian else '<H', value)[0]
        elif len(value) == 4:
            return struct.unpack('>I' if big_endian else '<I', value)[0]
        return None
    
    def get_string_value(self, tlv_entry):
        """Convert TLV value to string"""
        try:
            return tlv_entry['value'].decode('utf-8')
        except:
            return None
    
    def create_float_tlv(self, tlv_type, value, big_endian=True):
        """Create TLV with float value"""
        float_bytes = struct.pack('>f' if big_endian else '<f', value)
        length = struct.pack('>H', 4)
        return bytes([tlv_type]) + length + float_bytes
    
    def create_int_tlv(self, tlv_type, value, num_bytes=4, big_endian=True):
        """Create TLV with integer value"""
        if num_bytes == 1:
            int_bytes = struct.pack('B', value)
        elif num_bytes == 2:
            int_bytes = struct.pack('>H' if big_endian else '<H', value)
        elif num_bytes == 4:
            int_bytes = struct.pack('>I' if big_endian else '<I', value)
        else:
            raise ValueError("Unsupported byte count")
            
        length = struct.pack('>H', num_bytes)
        return bytes([tlv_type]) + length + int_bytes
    
    def create_string_tlv(self, tlv_type, value):
        """Create TLV with string value"""
        string_bytes = value.encode('utf-8')
        length = struct.pack('>H', len(string_bytes))
        return bytes([tlv_type]) + length + string_bytes

class PIDControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Robot Controller")
        self.root.geometry("1200x800")
        
        # Serial connection
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        
        # TLV Parser
        self.tlv_parser = TLVParser()
        
        # Data storage for plotting
        self.max_data_points = 500
        self.time_data = deque(maxlen=self.max_data_points)
        self.current_angle_data = deque(maxlen=self.max_data_points)
        self.target_angle_data = deque(maxlen=self.max_data_points)
        self.start_time = time.time()
        
        # Current parameter values
        self.current_target_angle = 0.0
        self.current_principle = 0.0
        self.current_integral = 0.0
        self.current_derivitive = 0.0
        self.current_deadband = 0.0
        self.current_basespeed = 0.0
        
        # Create GUI
        self.create_widgets()
        self.setup_plot()
        
        # Start plot animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
    def create_widgets(self):
        """Create the GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Serial connection frame
        serial_frame = ttk.LabelFrame(main_frame, text="Serial Connection")
        serial_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Port selection
        ttk.Label(serial_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        # Baud rate
        ttk.Label(serial_frame, text="Baud:").grid(row=0, column=2, padx=5, pady=5)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_var, 
                                  values=["9600", "57600", "115200", "230400"], width=10)
        baud_combo.grid(row=0, column=3, padx=5, pady=5)
        
        # Connect/Disconnect buttons
        self.connect_btn = ttk.Button(serial_frame, text="Connect", command=self.connect_serial)
        self.connect_btn.grid(row=0, column=4, padx=5, pady=5)
        
        self.disconnect_btn = ttk.Button(serial_frame, text="Disconnect", command=self.disconnect_serial, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=5, padx=5, pady=5)
        
        # Refresh ports button
        refresh_btn = ttk.Button(serial_frame, text="Refresh", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=6, padx=5, pady=5)
        
        # Status label
        self.status_label = ttk.Label(serial_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=7, padx=10, pady=5)
        
        # Content frame (side by side)
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left frame for controls
        left_frame = ttk.Frame(content_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # PID Parameters frame
        pid_frame = ttk.LabelFrame(left_frame, text="PID Parameters")
        pid_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Target Angle
        ttk.Label(pid_frame, text="Target Angle:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.target_angle_var = tk.DoubleVar(value=0.0)
        self.target_angle_entry = ttk.Entry(pid_frame, textvariable=self.target_angle_var, width=10)
        self.target_angle_entry.grid(row=0, column=1, padx=5, pady=2)
        
        # Principle parameter
        ttk.Label(pid_frame, text="Principle (P):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.principle_var = tk.DoubleVar(value=1.0)
        self.principle_entry = ttk.Entry(pid_frame, textvariable=self.principle_var, width=10)
        self.principle_entry.grid(row=1, column=1, padx=5, pady=2)
        
        # Integral parameter
        ttk.Label(pid_frame, text="Integral (I):").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.integral_var = tk.DoubleVar(value=0.0)
        self.integral_entry = ttk.Entry(pid_frame, textvariable=self.integral_var, width=10)
        self.integral_entry.grid(row=2, column=1, padx=5, pady=2)
        
        # Derivitive parameter
        ttk.Label(pid_frame, text="Derivitive (D):").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        self.derivitive_var = tk.DoubleVar(value=0.0)
        self.derivitive_entry = ttk.Entry(pid_frame, textvariable=self.derivitive_var, width=10)
        self.derivitive_entry.grid(row=3, column=1, padx=5, pady=2)
        
        # Deadband parameter
        ttk.Label(pid_frame, text="Deadband:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
        self.deadband_var = tk.DoubleVar(value=0.0)
        self.deadband_entry = ttk.Entry(pid_frame, textvariable=self.deadband_var, width=10)
        self.deadband_entry.grid(row=4, column=1, padx=5, pady=2)
        
        # Base speed
        ttk.Label(pid_frame, text="Base Speed:").grid(row=5, column=0, sticky=tk.W, padx=5, pady=2)
        self.basespeed_var = tk.DoubleVar(value=100.0)
        self.basespeed_entry = ttk.Entry(pid_frame, textvariable=self.basespeed_var, width=10)
        self.basespeed_entry.grid(row=5, column=1, padx=5, pady=2)
        
        # Buttons frame
        buttons_frame = ttk.Frame(pid_frame)
        buttons_frame.grid(row=6, column=0, columnspan=2, pady=10)
        
        # Send parameters button
        send_btn = ttk.Button(buttons_frame, text="Send All Parameters", command=self.send_all_parameters)
        send_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        # Get parameters button
        get_btn = ttk.Button(buttons_frame, text="Get Current Parameters", command=self.request_all_parameters)
        get_btn.pack(side=tk.LEFT, padx=(5, 0))
        
        # Current values frame
        values_frame = ttk.LabelFrame(left_frame, text="Current Values")
        values_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Current angle
        ttk.Label(values_frame, text="Current Angle:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.current_angle_label = ttk.Label(values_frame, text="--", foreground="blue")
        self.current_angle_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Target angle
        ttk.Label(values_frame, text="Target Angle:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.target_angle_display_label = ttk.Label(values_frame, text="--", foreground="green")
        self.target_angle_display_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Principle
        ttk.Label(values_frame, text="Principle:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.principle_display_label = ttk.Label(values_frame, text="--", foreground="red")
        self.principle_display_label.grid(row=2, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Integral
        ttk.Label(values_frame, text="Integral:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        self.integral_display_label = ttk.Label(values_frame, text="--", foreground="purple")
        self.integral_display_label.grid(row=3, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Derivitive
        ttk.Label(values_frame, text="Derivitive:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
        self.derivitive_display_label = ttk.Label(values_frame, text="--", foreground="orange")
        self.derivitive_display_label.grid(row=4, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Deadband
        ttk.Label(values_frame, text="Deadband:").grid(row=5, column=0, sticky=tk.W, padx=5, pady=2)
        self.deadband_display_label = ttk.Label(values_frame, text="--", foreground="brown")
        self.deadband_display_label.grid(row=5, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Base Speed
        ttk.Label(values_frame, text="Base Speed:").grid(row=6, column=0, sticky=tk.W, padx=5, pady=2)
        self.basespeed_display_label = ttk.Label(values_frame, text="--", foreground="navy")
        self.basespeed_display_label.grid(row=6, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Log frame
        log_frame = ttk.LabelFrame(left_frame, text="Log")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        # Log text widget with scrollbar
        log_scroll = ttk.Scrollbar(log_frame)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.log_text = tk.Text(log_frame, height=10, yscrollcommand=log_scroll.set)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        log_scroll.config(command=self.log_text.yview)
        
        # Right frame for plot
        right_frame = ttk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Plot frame
        self.plot_frame = ttk.LabelFrame(right_frame, text="Real-time Data")
        self.plot_frame.pack(fill=tk.BOTH, expand=True)
        
        # Initialize ports
        self.refresh_ports()
        
    def setup_plot(self):
        """Setup matplotlib plot"""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(8, 6))
        self.fig.tight_layout(pad=3.0)
        
        # Angle plot
        self.ax.set_title("Angle Tracking")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Angle (degrees)")
        self.ax.grid(True, alpha=0.3)
        self.line_current_angle, = self.ax.plot([], [], 'b-', label='Current Angle', linewidth=2)
        self.line_target_angle, = self.ax.plot([], [], 'g--', label='Target Angle', linewidth=2)
        self.ax.legend()
        
        # Embed plot in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.set(port_list[0])
        
    def connect_serial(self):
        """Connect to serial port"""
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.running = True
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()
            
            # Update UI
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_label.config(text="Connected", foreground="green")
            
            self.log_message(f"Connected to {port} at {baud} baud")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
            
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.running = False
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            
        # Update UI
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Disconnected", foreground="red")
        
        self.log_message("Disconnected from serial port")
        
    def read_serial_data(self):
        """Read data from serial port in separate thread"""
        buffer = bytearray()
        
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    
                    # Try to parse complete TLV structures
                    processed = 0
                    while processed < len(buffer):
                        # Need at least TLV header
                        if len(buffer) - processed < 3:
                            break
                            
                        # Get TLV length
                        tlv_length = struct.unpack('>H', buffer[processed+1:processed+3])[0]
                        total_tlv_size = 3 + tlv_length
                        
                        # Check if we have complete TLV
                        if processed + total_tlv_size > len(buffer):
                            break
                            
                        # Parse this TLV
                        tlv_data = buffer[processed:processed + total_tlv_size]
                        self.process_tlv_data(tlv_data)
                        
                        processed += total_tlv_size
                    
                    # Remove processed data from buffer
                    buffer = buffer[processed:]
                    
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                if self.running:  # Only log if we're still supposed to be running
                    self.log_message(f"Serial read error: {str(e)}")
                break
                
    def process_tlv_data(self, data):
        """Process received TLV data"""
        try:
            tlv_entries = self.tlv_parser.parse_tlv_data(data)
            current_time = time.time() - self.start_time
            
            for entry in tlv_entries:
                tlv_type = entry['type']
                
                if tlv_type == self.tlv_parser.TLV_TYPE_CURRENT_ANGLE:
                    angle = self.tlv_parser.get_float_value(entry)
                    if angle is not None:
                        self.time_data.append(current_time)
                        self.current_angle_data.append(angle)
                        self.root.after(0, lambda a=angle: self.current_angle_label.config(text=f"{a:.2f}°"))
                        
                elif tlv_type == self.tlv_parser.TLV_TYPE_GET_TARGET_ANGLE:
                    target = self.tlv_parser.get_float_value(entry)
                    if target is not None:
                        self.current_target_angle = target
                        # Extend target angle data to match current angle data length
                        while len(self.target_angle_data) < len(self.current_angle_data):
                            self.target_angle_data.append(target)
                        self.root.after(0, lambda t=target: self.target_angle_display_label.config(text=f"{t:.2f}°"))
                        self.root.after(0, lambda t=target: self.target_angle_var.set(t))
                        
                elif tlv_type == self.tlv_parser.TLV_TYPE_GET_PRINCIPLE:
                    principle = self.tlv_parser.get_float_value(entry)
                    if principle is not None:
                        self.current_principle = principle
                        self.root.after(0, lambda p=principle: self.principle_display_label.config(text=f"{p:.3f}"))
                        self.root.after(0, lambda p=principle: self.principle_var.set(p))
                        
                elif tlv_type == self.tlv_parser.TLV_TYPE_GET_INTEGRAL:
                    integral = self.tlv_parser.get_float_value(entry)
                    if integral is not None:
                        self.current_integral = integral
                        self.root.after(0, lambda i=integral: self.integral_display_label.config(text=f"{i:.3f}"))
                        self.root.after(0, lambda i=integral: self.integral_var.set(i))
                        
                elif tlv_type == self.tlv_parser.TLV_TYPE_GET_DERIVITIVE:
                    derivitive = self.tlv_parser.get_float_value(entry)
                    if derivitive is not None:
                        self.current_derivitive = derivitive
                        self.root.after(0, lambda d=derivitive: self.derivitive_display_label.config(text=f"{d:.3f}"))
                        self.root.after(0, lambda d=derivitive: self.derivitive_var.set(d))
                        
                elif tlv_type == self.tlv_parser.TLV_TYPE_GET_DEADBAND:
                    deadband = self.tlv_parser.get_float_value(entry)
                    if deadband is not None:
                        self.current_deadband = deadband
                        self.root.after(0, lambda db=deadband: self.deadband_display_label.config(text=f"{db:.3f}"))
                        self.root.after(0, lambda db=deadband: self.deadband_var.set(db))
                        
                elif tlv_type == self.tlv_parser.TLV_TYPE_GET_BASESPEED:
                    basespeed = self.tlv_parser.get_float_value(entry)
                    if basespeed is not None:
                        self.current_basespeed = basespeed
                        self.root.after(0, lambda bs=basespeed: self.basespeed_display_label.config(text=f"{bs:.1f}"))
                        self.root.after(0, lambda bs=basespeed: self.basespeed_var.set(bs))
                        
        except Exception as e:
            self.log_message(f"TLV parsing error: {str(e)}")
            
    def send_all_parameters(self):
        """Send all parameters to ESP32"""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return
            
        try:
            # Create TLV messages for each parameter
            target_tlv = self.tlv_parser.create_float_tlv(self.tlv_parser.TLV_TYPE_SET_TARGET_ANGLE, self.target_angle_var.get())
            principle_tlv = self.tlv_parser.create_float_tlv(self.tlv_parser.TLV_TYPE_SET_PRINCIPLE, self.principle_var.get())
            integral_tlv = self.tlv_parser.create_float_tlv(self.tlv_parser.TLV_TYPE_SET_INTEGRAL, self.integral_var.get())
            derivitive_tlv = self.tlv_parser.create_float_tlv(self.tlv_parser.TLV_TYPE_SET_DERIVITIVE, self.derivitive_var.get())
            deadband_tlv = self.tlv_parser.create_float_tlv(self.tlv_parser.TLV_TYPE_SET_DEADBAND, self.deadband_var.get())
            basespeed_tlv = self.tlv_parser.create_float_tlv(self.tlv_parser.TLV_TYPE_SET_BASESPEED, self.basespeed_var.get())
            
            # Send all parameters
            self.serial_port.write(target_tlv)
            self.serial_port.write(principle_tlv)
            self.serial_port.write(integral_tlv)
            self.serial_port.write(derivitive_tlv)
            self.serial_port.write(deadband_tlv)
            self.serial_port.write(basespeed_tlv)
            
            self.log_message(f"Sent Parameters: Target={self.target_angle_var.get():.1f}°, P={self.principle_var.get():.3f}, I={self.integral_var.get():.3f}, D={self.derivitive_var.get():.3f}, Deadband={self.deadband_var.get():.3f}, Speed={self.basespeed_var.get():.1f}")
            
        except Exception as e:
            messagebox.showerror("Send Error", f"Failed to send parameters: {str(e)}")
    
    def request_all_parameters(self):
        """Request all current parameters from ESP32"""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not Connected", "Please connect to serial port first")
            return
            
        try:
            # Send GET requests for all parameters (empty TLVs to request data)
            get_target_tlv = self.tlv_parser.create_int_tlv(self.tlv_parser.TLV_TYPE_GET_TARGET_ANGLE, 0, 1)
            get_principle_tlv = self.tlv_parser.create_int_tlv(self.tlv_parser.TLV_TYPE_GET_PRINCIPLE, 0, 1)
            get_integral_tlv = self.tlv_parser.create_int_tlv(self.tlv_parser.TLV_TYPE_GET_INTEGRAL, 0, 1)
            get_derivitive_tlv = self.tlv_parser.create_int_tlv(self.tlv_parser.TLV_TYPE_GET_DERIVITIVE, 0, 1)
            get_deadband_tlv = self.tlv_parser.create_int_tlv(self.tlv_parser.TLV_TYPE_GET_DEADBAND, 0, 1)
            get_basespeed_tlv = self.tlv_parser.create_int_tlv(self.tlv_parser.TLV_TYPE_GET_BASESPEED, 0, 1)
            
            # Send all GET requests
            self.serial_port.write(get_target_tlv)
            self.serial_port.write(get_principle_tlv)
            self.serial_port.write(get_integral_tlv)
            self.serial_port.write(get_derivitive_tlv)
            self.serial_port.write(get_deadband_tlv)
            self.serial_port.write(get_basespeed_tlv)
            
            self.log_message("Requested all current parameters from ESP32")
            
        except Exception as e:
            messagebox.showerror("Request Error", f"Failed to request parameters: {str(e)}")
            
    def update_plot(self, frame):
        """Update the matplotlib plot"""
        if len(self.time_data) < 2:
            return
            
        time_array = list(self.time_data)
        
        # Update current angle plot
        if len(self.current_angle_data) > 0:
            self.line_current_angle.set_data(time_array[-len(self.current_angle_data):], list(self.current_angle_data))
            
        # Update target angle plot (extend target to match current data length)
        if len(self.target_angle_data) > 0:
            # Make sure target angle data matches current angle data length
            while len(self.target_angle_data) < len(self.current_angle_data):
                self.target_angle_data.append(self.current_target_angle)
            
            target_time = time_array[-len(self.target_angle_data):]
            self.line_target_angle.set_data(target_time, list(self.target_angle_data))
            
        # Auto-scale axes
        if time_array:
            self.ax.relim()
            self.ax.autoscale_view()
            
            # Keep last 30 seconds visible
            if time_array[-1] > 30:
                self.ax.set_xlim(time_array[-1] - 30, time_array[-1])
                    
    def log_message(self, message):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"
        
        # Thread-safe GUI update
        self.root.after(0, lambda: self._add_to_log(full_message))
        
    def _add_to_log(self, message):
        """Add message to log text widget (called from main thread)"""
        self.log_text.insert(tk.END, message)
        self.log_text.see(tk.END)
        
        # Limit log size
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 1000:
            self.log_text.delete('1.0', '500.0')

def main():
    root = tk.Tk()
    app = PIDControllerGUI(root)
    
    def on_closing():
        app.disconnect_serial()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()