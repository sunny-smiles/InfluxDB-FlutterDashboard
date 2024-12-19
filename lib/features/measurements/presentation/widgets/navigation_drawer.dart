// lib/features/measurements/presentation/widgets/navigation_drawer.dart
import 'package:flutter/material.dart';
import '../../domain/enums/measurement_type.dart';
import '../pages/home_page.dart';
import '../pages/measurement_page.dart';

class NavigationDrawer extends StatelessWidget {
  const NavigationDrawer({super.key});

  @override
  Widget build(BuildContext context) {
    return Drawer(
      child: Container(
        color: Colors.grey[900],
        child: ListView(
          padding: EdgeInsets.zero,
          children: [
            InkWell(
              onTap: () {
                Navigator.pushAndRemoveUntil(
                  context,
                  MaterialPageRoute(builder: (context) => const HomePage()),
                  (route) => false,
                );
              },
              child: DrawerHeader(
                decoration: const BoxDecoration(
                  color: Colors.blue,
                ),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: const [
                    Text(
                      'Measurements',
                      style: TextStyle(
                        color: Colors.white,
                        fontSize: 24,
                      ),
                    ),
                    SizedBox(height: 8),
                    Text(
                      'Click here to return home',
                      style: TextStyle(
                        color: Colors.white70,
                        fontSize: 14,
                      ),
                    ),
                  ],
                ),
              ),
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.explore,
              iconColor: Colors.lightBlue[600]!,
              title: 'Heading Data',
              measurementType: MeasurementType.heading,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.location_on,
              iconColor: Colors.green[600]!,
              title: 'Latitude Data',
              measurementType: MeasurementType.latitude,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.location_searching,
              iconColor: Colors.orange[600]!,
              title: 'Longitude Data',
              measurementType: MeasurementType.longitude,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.compress,
              iconColor: Colors.yellow[700]!,
              title: 'Ultrasonic-1-Pressure',
              measurementType: MeasurementType.ultrasonic1pressure,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.speed,
              iconColor: Colors.brown[500]!,
              title: 'Ultrasonic-1-Speed',
              measurementType: MeasurementType.ultrasonic1speed,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.compress,
              iconColor: Colors.deepPurple[500]!,
              title: 'Ultrasonic-2-Pressure',
              measurementType: MeasurementType.ultrasonic2pressure,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.speed,
              iconColor: Colors.red[600]!,
              title: 'Ultrasonic-2-Speed',
              measurementType: MeasurementType.ultrasonic2speed,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.settings_input_component,
              iconColor: Colors.amber[700]!,
              title: 'DAC',
              measurementType: MeasurementType.dac,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.device_thermostat,
              iconColor: Colors.blueGrey[500]!,
              title: 'INA219',
              measurementType: MeasurementType.ina219,
            ),
            _buildNavigationItem(
              context: context,
              icon: Icons.bolt,
              iconColor: Colors.deepOrange[600]!,
              title: 'ACS758',
              measurementType: MeasurementType.acs758,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildNavigationItem({
    required BuildContext context,
    required IconData icon,
    required Color iconColor,
    required String title,
    required MeasurementType measurementType,
  }) {
    return ListTile(
      leading: Icon(
        icon,
        color: iconColor,
      ),
      title: Text(
        title,
        style: const TextStyle(color: Colors.white),
      ),
      onTap: () {
        Navigator.pop(context);
        Navigator.push(
          context,
          MaterialPageRoute(
            builder: (context) => MeasurementPage(
              measurementType: measurementType,
              title: title,
            ),
          ),
        );
      },
    );
  }
}