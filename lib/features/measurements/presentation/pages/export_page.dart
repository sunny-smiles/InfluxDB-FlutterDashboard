import 'package:flutter/material.dart';
import '../../data/datasources/influxdb_datasource.dart'; // Adjust with the correct path
import '../../domain/enums/measurement_type.dart';
import '../../../../core/services/file_export_service.dart'; // Import the FileService

class ExportPage extends StatefulWidget {
  @override
  _ExportPageState createState() => _ExportPageState();
}

class _ExportPageState extends State<ExportPage> {
  MeasurementType _selectedType = MeasurementType.heading; // Default measurement type
  String _selectedTimeRange = '1m'; // Default time range
  final InfluxDBDataSource _dataSource = InfluxDBDataSource();
  bool _isLoading = false; // To handle loading state

  // Helper to show snackbars
  void _showSnackBar(String message, {bool isError = false}) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message, style: const TextStyle(color: Colors.white)),
        backgroundColor: isError ? Colors.red : Colors.green,
      ),
    );
  }

  // Function to handle export action
  Future<void> _exportData() async {
    setState(() {
      _isLoading = true; // Show loading indicator
    });

    try {
      // Fetch data from the data source
      final data = await _dataSource.exportData(_selectedType, _selectedTimeRange);

      if (data.isEmpty) {
        _showSnackBar('No data available for export.', isError: true);
      } else {
        // Save the data to a CSV file using FileService
        final filePath = await FileService.saveCSV(data);

        // Notify the user of success
        _showSnackBar('Data exported successfully!\nSaved to: $filePath');
      }
    } catch (e) {
      _showSnackBar('Failed to export data: $e', isError: true);
    } finally {
      setState(() {
        _isLoading = false; // Hide loading indicator
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Export Data', style: TextStyle(color: Colors.white))),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Select Measurement Type:',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold, color: Colors.white),
            ),
            DropdownButton<MeasurementType>(
              dropdownColor: Colors.black, // Set dropdown background to black for contrast
              value: _selectedType,
              items: MeasurementType.values.map((MeasurementType type) {
                return DropdownMenuItem<MeasurementType>(
                  value: type,
                  child: Text(type.toString().split('.').last, style: const TextStyle(color: Colors.white)),
                );
              }).toList(),
              onChanged: (MeasurementType? newValue) {
                setState(() {
                  _selectedType = newValue!;
                });
              },
            ),
            const SizedBox(height: 20),
            const Text(
              'Select Time Range:',
              style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold, color: Colors.white),
            ),
            DropdownButton<String>(
              dropdownColor: Colors.black, // Set dropdown background to black for contrast
              value: _selectedTimeRange,
              items: ['1m', '5m', '1h', '12h', '1d'].map((String timeRange) {
                return DropdownMenuItem<String>(
                  value: timeRange,
                  child: Text(timeRange, style: const TextStyle(color: Colors.white)),
                );
              }).toList(),
              onChanged: (String? newValue) {
                setState(() {
                  _selectedTimeRange = newValue!;
                });
              },
            ),
            const SizedBox(height: 20),
            Center(
              child: ElevatedButton(
                onPressed: _isLoading ? null : _exportData,
                child: _isLoading
                    ? const CircularProgressIndicator(
                        valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
                      )
                    : const Text('Export Data', style: TextStyle(color: Colors.black)),
              ),
            ),
          ],
        ),
      ),
      backgroundColor: Colors.black, // Set background color to black for better contrast
    );
  }
}
