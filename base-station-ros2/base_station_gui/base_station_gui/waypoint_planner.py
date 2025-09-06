# Interactive Waypoint Mission Planner Created by Brighton Anderson May 2025
 
import tkinter
import tkinter.messagebox
import tkinter.simpledialog
from tkinter import ttk
from tkinter import filedialog
import tkintermapview
import os
import math
import yaml # Added for YAML saving

# --- Constants for Presets ---
PRESET_ORIGINS = {
    "UL Provo Marina": {"lat": 40.238947, "lon": -111.739538, "alt": 1365.4333},
    "Spanish Oaks": {"lat": 40.0717592, "lon": -111.6001751, "alt": 1564.568},
}

class App:
    def __init__(self, root_widget):
        self.root_widget = root_widget
        self.root_widget.title("Waypoint Mission Planner (ENU & Depth)")
        self.root_widget.geometry("1100x750") # Increased size slightly for more info

        # --- Data Storage ---
        self.origin_data = None  # Will store {'lat': float, 'lon': float, 'alt': float, 'map_marker_object': object}
        self.waypoints = []      # List of {'east': float, 'north': float, 'up': float, 'depth': float,
                                 #         'original_lat': float, 'original_lon': float, 'map_marker_object': object}

        # --- UI Elements ---
        self.control_frame = ttk.Frame(self.root_widget, padding=10)
        self.control_frame.pack(side=tkinter.LEFT, fill=tkinter.Y)

        ttk.Label(self.control_frame, text="Mission Origin", font=("Arial", 14, "bold")).pack(pady=(0, 5))
        self.set_origin_button = ttk.Button(self.control_frame, text="Set Origin", command=self.prompt_origin_choice)
        self.set_origin_button.pack(pady=5, fill=tkinter.X)
        self.origin_info_label = ttk.Label(self.control_frame, text="Origin not set.", wraplength=320) # Increased wraplength
        self.origin_info_label.pack(pady=5)

        ttk.Label(self.control_frame, text="Waypoints (ENU relative to Origin)", font=("Arial", 14, "bold")).pack(pady=(10, 5))
        self.waypoints_listbox = tkinter.Listbox(self.control_frame, height=15, width=50) # Increased width
        self.waypoints_listbox.pack(pady=5, fill=tkinter.BOTH, expand=True)
        
        self.clear_waypoints_button = ttk.Button(self.control_frame, text="Clear All Waypoints & Origin", command=self.clear_mission_confirmation)
        self.clear_waypoints_button.pack(pady=5, fill=tkinter.X)

        ttk.Label(self.control_frame, text="Mission Actions", font=("Arial", 14, "bold")).pack(pady=(10, 5))
        self.save_mission_button = ttk.Button(self.control_frame, text="Save Mission to .yaml File", command=self.save_mission_to_yaml)
        self.save_mission_button.pack(pady=5, fill=tkinter.X)

        self.map_frame = ttk.Frame(self.root_widget)
        self.map_frame.pack(side=tkinter.RIGHT, fill=tkinter.BOTH, expand=True)

        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, width=700, height=700, corner_radius=0)
        self.map_widget.pack(fill=tkinter.BOTH, expand=True)

        # Set to Google Satellite tile server
        # IMPORTANT: Please be aware of the Terms of Service for any tile server you use, including Google's.
        # Direct tile access may be against their ToS if not used with their official APIs.
        # This URL is commonly used in tkintermapview examples.
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)


        self.map_widget.set_position(40.2338, -111.6585) # Provo, UT
        self.map_widget.set_zoom(10) # Adjusted default zoom

        self.map_widget.add_right_click_menu_command(label="Add Waypoint Here",
                                                     command=self.add_waypoint_on_map_click,
                                                     pass_coords=True)
        
        self._update_ui_state()

    def _geodetic_to_enu(self, lat_wp_deg, lon_wp_deg, alt_wp, lat_origin_deg, lon_origin_deg, alt_origin):
        """
        Converts Geodetic coordinates (latitude, longitude, altitude) to local ENU (East, North, Up)
        coordinates relative to a given origin.
        Assumes WGS84 ellipsoid.
        """
        R_EARTH = 6378137.0  # WGS84 major axis
        F_INV = 298.257223563 # WGS84 inverse flattening
        E_SQ = 1 - (1 - 1/F_INV)**2 # Square of eccentricity

        lat_wp_rad = math.radians(lat_wp_deg)
        lon_wp_rad = math.radians(lon_wp_deg)
        lat_origin_rad = math.radians(lat_origin_deg)
        lon_origin_rad = math.radians(lon_origin_deg)

        # Convert LLA to ECEF
        def lla_to_ecef(lat_r, lon_r, h):
            sin_lat = math.sin(lat_r)
            cos_lat = math.cos(lat_r)
            N = R_EARTH / math.sqrt(1 - E_SQ * sin_lat**2)
            
            x = (N + h) * cos_lat * math.cos(lon_r)
            y = (N + h) * cos_lat * math.sin(lon_r)
            z = (N * (1 - E_SQ) + h) * sin_lat
            return x, y, z

        x_origin_ecef, y_origin_ecef, z_origin_ecef = lla_to_ecef(lat_origin_rad, lon_origin_rad, alt_origin)
        x_wp_ecef, y_wp_ecef, z_wp_ecef = lla_to_ecef(lat_wp_rad, lon_wp_rad, alt_wp)

        # Vector difference in ECEF
        dx = x_wp_ecef - x_origin_ecef
        dy = y_wp_ecef - y_origin_ecef
        dz = z_wp_ecef - z_origin_ecef

        # Rotate ECEF difference vector to ENU
        sin_lat_o = math.sin(lat_origin_rad)
        cos_lat_o = math.cos(lat_origin_rad)
        sin_lon_o = math.sin(lon_origin_rad)
        cos_lon_o = math.cos(lon_origin_rad)

        east = -sin_lon_o * dx + cos_lon_o * dy
        north = -sin_lat_o * cos_lon_o * dx - sin_lat_o * sin_lon_o * dy + cos_lat_o * dz
        up = cos_lat_o * cos_lon_o * dx + cos_lat_o * sin_lon_o * dy + sin_lat_o * dz
        
        return east, north, up

    def _update_ui_state(self):
        if self.origin_data:
            self.save_mission_button.config(state=tkinter.NORMAL)
            self.clear_waypoints_button.config(state=tkinter.NORMAL) # Enable clearing if origin or waypoints exist
        else:
            self.save_mission_button.config(state=tkinter.DISABLED)
            if not self.waypoints: # Only disable clear if both origin and waypoints are empty
                self.clear_waypoints_button.config(state=tkinter.DISABLED)
            else: # If there are waypoints (e.g. from a loaded file in future), allow clearing.
                 self.clear_waypoints_button.config(state=tkinter.NORMAL)


    def prompt_origin_choice(self):
        """Prompts user to choose an origin source (preset or manual)."""
        dialog = tkinter.Toplevel(self.root_widget)
        dialog.title("Set Mission Origin")
        dialog.geometry("350x200")
        dialog.transient(self.root_widget) # Make it modal
        dialog.grab_set() # Ensure input focus

        ttk.Label(dialog, text="Select an origin source:").pack(pady=10)

        var = tkinter.StringVar(value="Manual Entry") # Default selection

        options = list(PRESET_ORIGINS.keys()) + ["Manual Entry"]
        for option_text in options:
            rb = ttk.Radiobutton(dialog, text=option_text, variable=var, value=option_text)
            rb.pack(anchor=tkinter.W, padx=20)
        
        def on_ok():
            choice = var.get()
            dialog.destroy()
            if choice == "Manual Entry":
                self.prompt_manual_origin()
            else:
                preset_data = PRESET_ORIGINS[choice]
                self._set_origin(preset_data['lat'], preset_data['lon'], preset_data['alt'], choice)

        ok_button = ttk.Button(dialog, text="OK", command=on_ok)
        ok_button.pack(pady=10)
        
        # Center dialog
        self.root_widget.eval(f'tk::PlaceWindow {str(dialog)} center')


    def prompt_manual_origin(self):
        """Prompts the user for manual origin LLA details."""
        lla_str = tkinter.simpledialog.askstring(
            "Manual Origin Input",
            "Enter Origin Latitude, Longitude, Altitude (comma-separated):\n(e.g., 40.2385,-111.7390,1400.0)",
            parent=self.root_widget
        )
        if lla_str is None: return

        try:
            parts = lla_str.split(',')
            if len(parts) != 3:
                raise ValueError("Input must be three comma-separated values.")
            lat = float(parts[0].strip())
            lon = float(parts[1].strip())
            alt = float(parts[2].strip())
        except ValueError as e:
            tkinter.messagebox.showerror("Invalid Input", f"Please enter valid numbers for Lat, Lon, Alt.\nError: {e}")
            return
        except Exception as e:
            tkinter.messagebox.showerror("Error", f"An unexpected error occurred: {e}")
            return
        
        self._set_origin(lat, lon, alt, "Manual")

    def _set_origin(self, lat, lon, alt, origin_name="Origin"):
        if self.origin_data or self.waypoints:
            if not tkinter.messagebox.askyesno("Confirm New Origin",
                                               "Setting a new origin will clear all existing waypoints. Continue?"):
                return
            self.clear_mission_data(show_confirmation=False) # Clear without extra message

        self.origin_data = {'lat': lat, 'lon': lon, 'alt': alt, 'name': origin_name}
        
        origin_marker_text = f"{origin_name}\nLat:{lat:.4f}\nLon:{lon:.4f}\nAlt:{alt:.1f}m"
        if origin_name in PRESET_ORIGINS: # Shorter text for known presets on map
            origin_marker_text = origin_name

        origin_map_marker = self.map_widget.set_marker(lat, lon, text=origin_marker_text, text_color="white", marker_color_circle="blue", font=("Arial", 10, "bold"))
        self.origin_data['map_marker_object'] = origin_map_marker

        self.origin_info_label.config(text=f"Origin ({origin_name}):\nLat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
        self.map_widget.set_position(lat, lon)
        self.map_widget.set_zoom(16) # Zoom closer for origin
        
        self._update_waypoint_display()
        self._update_ui_state()
        tkinter.messagebox.showinfo("Origin Set", f"Origin '{origin_name}' set successfully. Right-click map to add waypoints.")


    def add_waypoint_on_map_click(self, coords):
        if not self.origin_data:
            tkinter.messagebox.showwarning("Set Origin First", "Please set the mission origin before adding waypoints.")
            return

        clicked_lat, clicked_lon = coords
        wp_index = len(self.waypoints) + 1

        depth_str = tkinter.simpledialog.askstring("Waypoint Depth", f"Enter Depth for Waypoint {wp_index} (meters, positive down):", parent=self.root_widget)
        if depth_str is None: return # User cancelled
        try:
            depth = float(depth_str)
            if depth < 0:
                tkinter.messagebox.showerror("Invalid Input", "Depth must be a non-negative number.")
                return
        except ValueError:
            tkinter.messagebox.showerror("Invalid Input", "Please enter a valid number for depth.")
            return

        # For ENU calculation, assume waypoint is at the same altitude as origin. 'up_enu' will be 0.
        # The 'depth' is an additional parameter.
        origin_lat = self.origin_data['lat']
        origin_lon = self.origin_data['lon']
        origin_alt = self.origin_data['alt']

        east, north, up_enu = self._geodetic_to_enu(clicked_lat, clicked_lon, origin_alt, # wp alt = origin alt for this
                                                    origin_lat, origin_lon, origin_alt)

        map_marker = self.map_widget.set_marker(clicked_lat, clicked_lon, 
                                               text=f"WP{wp_index}\nD:{depth:.1f}m",
                                               text_color="white", 
                                               marker_color_circle="green",
                                               font=("Arial", 10, "bold"))
        
        self.waypoints.append({
            'id': wp_index,
            'x': east, 
            'y': north, 
            'z': up_enu, 
            'depth': depth,
            'original_lat': clicked_lat, 
            'original_lon': clicked_lon,
            'map_marker_object': map_marker
        })
        self._update_waypoint_display()
        self._update_ui_state()

    def _update_waypoint_display(self):
        self.waypoints_listbox.delete(0, tkinter.END)
        if self.origin_data:
            self.waypoints_listbox.insert(tkinter.END, 
                f"ORIGIN ({self.origin_data.get('name', 'N/A')}): Lat={self.origin_data['lat']:.6f}, Lon={self.origin_data['lon']:.6f}, Alt={self.origin_data['alt']:.2f}m")
            self.waypoints_listbox.itemconfig(tkinter.END, {'fg': 'blue'})


        for wp in self.waypoints:
            display_text = (f"WP {wp['id']}: X={wp['x']:.2f}, Y={wp['y']:.2f}, Z={wp['z']:.2f}m | D={wp['depth']:.1f}m "
                            f"(Geo: {wp['original_lat']:.5f}, {wp['original_lon']:.5f})")
            self.waypoints_listbox.insert(tkinter.END, display_text)
    
    def clear_mission_confirmation(self):
        if not self.origin_data and not self.waypoints:
            tkinter.messagebox.showinfo("No Data", "Nothing to clear.")
            return
        if tkinter.messagebox.askyesno("Confirm Clear", "Clear current origin and all waypoints? This cannot be undone."):
            self.clear_mission_data(show_confirmation=True)

    def clear_mission_data(self, show_confirmation=True):
        self.map_widget.delete_all_marker()

        self.origin_data = None
        self.waypoints = []
        
        self.origin_info_label.config(text="Origin not set.")
        self.map_widget.set_position(40.2338, -111.6585) 
        self.map_widget.set_zoom(10)
        
        self._update_waypoint_display()
        self._update_ui_state()
        if show_confirmation:
             tkinter.messagebox.showinfo("Cleared", "Mission data and waypoints cleared.")


    def save_mission_to_yaml(self):
        if not self.origin_data:
            tkinter.messagebox.showerror("Error", "Cannot save mission. Origin not set.")
            return
        if not self.waypoints:
            tkinter.messagebox.showwarning("No Waypoints", "Mission has an origin but no waypoints. Save anyway?")
            # Or decide to prevent saving if no waypoints:
            # tkinter.messagebox.showerror("Error", "No waypoints to save.")
            # return


        filepath = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML Mission Files", "*.yaml"), ("All Files", "*.*")],
            title="Save Mission YAML File",
            initialfile="mission_plan.yaml" # Suggest a default name
        )

        if not filepath:
            return # User cancelled

        # Derive mission name from filename
        base_name = os.path.basename(filepath)
        mission_name = base_name.lower().replace("_mission.yaml", "").replace(".yaml", "")
        if not mission_name: # if somehow ended up with just ".yaml" or "_mission.yaml"
            mission_name = "default_mission"

        # Ensure the filename ends with _mission.yaml if it's not already
        if not filepath.lower().endswith("_mission.yaml"):
            if filepath.lower().endswith(".yaml"):
                filepath = filepath[:-5] + "_mission.yaml"
            else:
                filepath = filepath + "_mission.yaml"


        mission_content = {
            "mission_type": "waypoint",
            "mission_name": mission_name,
            "origin_lla": {
                "latitude": self.origin_data['lat'],
                "longitude": self.origin_data['lon'],
                "altitude": self.origin_data['alt'], # WGS84 altitude
                "name": self.origin_data.get('name', "Manual")
            },
            "reference_frame_for_waypoints": "ENU_at_origin_LLA",
            "waypoints": []
        }

        for wp in self.waypoints:
            mission_content["waypoints"].append({
                "id": wp['id'],
                "position_enu": { # Relative to origin_lla
                    "x": round(wp['x'], 3),  # meters
                    "y": round(wp['y'], 3), # meters
                    "z": round(wp['z'], 3)      # meters (height relative to origin's altitude)
                },
                "depth": round(wp['depth'], 2),    # meters (positive downwards from waypoint's ENU position)
                "original_map_click_lla": { # For reference
                    "latitude": round(wp['original_lat'], 7),
                    "longitude": round(wp['original_lon'], 7)
                }
            })
            
        try:
            with open(filepath, 'w') as f:
                yaml.dump(mission_content, f, sort_keys=False, indent=2, default_flow_style=False)
            tkinter.messagebox.showinfo("Success", f"Mission saved to {os.path.basename(filepath)}")
        except Exception as e:
            tkinter.messagebox.showerror("File Save Error", f"Could not save mission YAML file: {e}")