import omni.ext
import omni.ui as ui
import random as rd
import threading
import time
import omni.kit.commands
import info_panel_ext

class InfoPanelExtExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[info_panel_ext] info_panel_ext startup")

        self._window = ui.Window("Info Panel", width=300, height=300)
        self._is_running = False  # Flag to track if the timer is running

        with self._window.frame:
            with ui.VStack():    
                with ui.HStack():
                    with ui.VStack():
                        # Initialize labels
                        self.time_label = ui.Label("", height=0)
                        self.distance_label = ui.Label("Distance Travelled: 0 meters", height=0)
                        self.battery_label = ui.Label("Battery: 100 %", height=0)
                        self.temp_label = ui.Label("Temperature: 0 Celcius", height=0)
                        self.storage_label = ui.Label("Data Storage: 0 %", height=0)
                        self.transfer_label = ui.Label("Data Transfer? False", height=0)
                        self.satellite_label = ui.Label("Satellite In Sight: False", height=0)

                        def start_simulation():
                            omni.kit.commands.execute('ToolbarPlayButtonClicked')
                            if not self._is_running:
                                self._is_running = True
                                self.start_time = time.time()
                                threading.Thread(target=update_values_periodically).start()

                        def reset_simulation():
                            self._is_running = False
                            self.time_label.text = "Time Resetting"
                            omni.kit.commands.execute('ToolbarStopButtonClicked')

                        def update_values_periodically():
                            while self._is_running:
                                current_time = (time.time() - self.start_time) * 100
                                hour = int(current_time / 3600)
                                min = int(current_time / 60) % 60
                                sec = int(current_time) % 60
                                self.time_label.text = f"Time: {hour:02}:{min:02}:{sec:02}"
                                self.distance_label.text = f"Distance Travelled: {rd.randint(1, 1000)} meters"
                                self.battery_label.text = f"Battery: {rd.randint(0, 100)} %"
                                self.temp_label.text = f"Temperature: {rd.randint(-20, 40)} Celcius"
                                self.storage_label.text = f"Data Storage: {rd.randint(0, 100)} %"
                                self.transfer_label.text = f"Data Transfer? {bool(rd.randint(0, 1))}"
                                self.satellite_label.text = f"Satellite In Sight: {bool(rd.randint(0, 1))}"
                                time.sleep(1)  # Sleep for 10 seconds before the next update

                    ui.Image("c:/Users/olinl/Desktop/Moonwalker/jplimg.png")
                ui.Button("Start Simulation", clicked_fn=start_simulation)
                ui.Button("Reset", clicked_fn=reset_simulation)

    def on_shutdown(self):
        print("[info_panel_ext] info_panel_ext shutdown")
