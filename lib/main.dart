import 'dart:async';
import 'package:flutter/material.dart';
import 'package:syncfusion_flutter_charts/charts.dart';
import 'package:influxdb_client/api.dart';
import 'package:intl/intl.dart';
import 'package:flutter_dotenv/flutter_dotenv.dart';

void main() {
  dotenv.load(); // Load environments
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'InfluxDB Data Visualizer',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: InfluxDBHome(),
    );
  }
}

class InfluxDBHome extends StatefulWidget {
  @override
  _InfluxDBHomeState createState() => _InfluxDBHomeState();
}

class _InfluxDBHomeState extends State<InfluxDBHome> {
  final String _url = dotenv.env['_URL'] ?? ''; // Your InfluxDB server URL
  final String _token = dotenv.env['_TOKEN'] ?? ''; // Your InfluxDB token
  final String _org = dotenv.env['_ORG'] ?? ''; // Your InfluxDB organization
  final String _bucket = dotenv.env['_BUCKET'] ?? ''; // Your InfluxDB bucket

  List<_ChartData> _headingData = []; // For heading data
  List<_ChartData> _latitudeData = []; // For latitude data
  List<_ChartData> _longitudeData = []; // For longitude data
  Timer? _timer; // Timer for fetching data

  @override
  void initState() {
    super.initState();
    _fetchData(); // Fetch initial data
    _startFetching(); // Start the timer to fetch data every 10 seconds
  }

  void _startFetching() {
    _timer = Timer.periodic(Duration(seconds: 10), (timer) {
      _fetchData(); // Fetch data every 10 seconds
    });
  }

  Future<void> _fetchData() async {
    var client = InfluxDBClient(
      url: _url,
      token: _token,
      org: _org,
      bucket: _bucket,
    );

    var queryService = client.getQueryService();

    // Fetch heading data from compass_dummydata
    var headingQuery = '''
      from(bucket: "$_bucket")
      |> range(start: -1m) // Last 1 minute
      |> filter(fn: (r) => r["_measurement"] == "compass_dummydata")
      |> filter(fn: (r) => r["_field"] == "heading")
      |> filter(fn: (r) => r["sensor_id"] == "compass")
      |> last()
    ''';

    // Fetch latitude data from gps_dummydata
    var latitudeQuery = '''
      from(bucket: "$_bucket")
      |> range(start: -1m) // Last 1 minute
      |> filter(fn: (r) => r["_measurement"] == "gps_dummydata")
      |> filter(fn: (r) => r["_field"] == "latitude")
      |> filter(fn: (r) => r["sensor_id"] == "gps")
      |> last()
    ''';

    // Fetch longitude data from gps_dummydata
    var longitudeQuery = '''
      from(bucket: "$_bucket")
      |> range(start: -1m) // Last 1 minute
      |> filter(fn: (r) => r["_measurement"] == "gps_dummydata")
      |> filter(fn: (r) => r["_field"] == "longitude")
      |> filter(fn: (r) => r["sensor_id"] == "gps")
      |> last()
    ''';

    // Execute heading query
    var headingStream = await queryService.query(headingQuery);
    await headingStream.forEach((record) {
      DateTime utcTimestamp = DateTime.parse(record['_time']);
      DateTime localTimestamp = utcTimestamp.toLocal();
      double headingValue = record['_value'].toDouble();
      setState(() {
        _headingData.add(_ChartData(localTimestamp, headingValue));
        // Keep only the last 10 data points
        if (_headingData.length > 10) {
          _headingData = _headingData.sublist(_headingData.length - 10);
        }
      });
    });

    // Execute latitude query
    var latitudeStream = await queryService.query(latitudeQuery);
    await latitudeStream.forEach((record) {
      DateTime utcTimestamp = DateTime.parse(record['_time']);
      DateTime localTimestamp = utcTimestamp.toLocal();
      double latitudeValue = record['_value'].toDouble();
      setState(() {       
        _latitudeData.add(_ChartData(localTimestamp, latitudeValue));
        // Keep only the last 10 data points
        if (_latitudeData.length > 10) {
          _latitudeData = _latitudeData.sublist(_latitudeData.length - 10);
        }
      });
    });

    // Execute longitude query
    var longitudeStream = await queryService.query(longitudeQuery);
    await longitudeStream.forEach((record) {
      DateTime utcTimestamp = DateTime.parse(record['_time']);
      DateTime localTimestamp = utcTimestamp.toLocal();
      double longitudeValue = record['_value'].toDouble();

      print('Fetched value: $longitudeValue at UTC: $utcTimestamp, Local: $localTimestamp');
      
      setState(() {       
        _longitudeData.add(_ChartData(localTimestamp, longitudeValue));
        // Keep only the last 10 data points
        if (_longitudeData.length > 10) {
          _longitudeData = _longitudeData.sublist(_longitudeData.length - 10);
        }
      });
    });

    client.close();
  }

  @override
  void dispose() {
    _timer?.cancel(); // Cancel the timer when disposing
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black, // Set the background color to black
      appBar: AppBar(
        title: Text(
          'InfluxDB Data Visualizer', 
          style: TextStyle(color: Colors.white),
        ),
        backgroundColor: Colors.grey[900], // Darker app bar
      ),
      body: SingleChildScrollView(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Heading Data Chart
            Container(
              height: 300,
              child: SfCartesianChart(
                title: ChartTitle(text: 'Heading Data Over Time', textStyle: TextStyle(color: Colors.white)),
                primaryXAxis: DateTimeAxis(
                  dateFormat: DateFormat('yyyy-MM-dd HH:mm:ss.SSS'),
                  labelStyle: TextStyle(color: Colors.white), // Set axis label color to white
                ),
                primaryYAxis: NumericAxis(
                  labelStyle: TextStyle(color: Colors.white), // Set y-axis label color to white
                ),
                tooltipBehavior: TooltipBehavior(
                  enable: false,
                  header: '',
                  format: 'point.x\nValue: point.y',
                  canShowMarker: true,
                  textStyle: TextStyle(color: Colors.white),
                ),
                trackballBehavior: TrackballBehavior(
                  enable: true,
                  tooltipSettings: InteractiveTooltip(
                    enable: true,
                    color: Colors.red,
                  )
                ),
                series: <CartesianSeries>[
                  LineSeries<_ChartData, DateTime>(
                    dataSource: _headingData,
                    xValueMapper: (_ChartData data, _) => data.timestamp,
                    yValueMapper: (_ChartData data, _) => data.value,
                    markerSettings: MarkerSettings(isVisible: false),
                    dataLabelSettings: DataLabelSettings(
                      isVisible: true,
                      textStyle: TextStyle(color: Colors.white),
                    ),
                    color: Colors.blue, // Customize line color
                    
                  ),
                ],
              ),
            ),
            
            SizedBox(height: 20),

            // Latitude Data Chart
            Container(
              height: 300,
              child: SfCartesianChart(
                title: ChartTitle(text: 'Latitude Data Over Time', textStyle: TextStyle(color: Colors.white)),
                primaryXAxis: DateTimeAxis(
                  dateFormat: DateFormat('yyyy-MM-dd HH:mm:ss'),
                  labelStyle: TextStyle(color: Colors.white), // Set axis label color to white
                ),
                primaryYAxis: NumericAxis(
                  labelStyle: TextStyle(color: Colors.white), // Set y-axis label color to white
                ),
                tooltipBehavior: TooltipBehavior(
                  enable: true,
                  header: '',
                  format: 'point.x\nValue: point.y',
                  canShowMarker: true,
                  textStyle: TextStyle(color: Colors.white),
                ),
                series: <CartesianSeries>[
                  LineSeries<_ChartData, DateTime>(
                    dataSource: _latitudeData,
                    xValueMapper: (_ChartData data, _) => data.timestamp,
                    yValueMapper: (_ChartData data, _) => data.value,
                    markerSettings: MarkerSettings(isVisible: true),
                    dataLabelSettings: DataLabelSettings(
                      isVisible: true,
                      textStyle: TextStyle(color: Colors.white),
                    ),
                    color: Colors.green, // Customize line color
                  ),
                ],
              ),
            ),
            SizedBox(height: 20),

            // Longitude Data Chart
            Container(
              height: 300,
              child: SfCartesianChart(
                title: ChartTitle(text: 'Longitude Data Over Time', textStyle: TextStyle(color: Colors.white)),
                primaryXAxis: DateTimeAxis(
                  dateFormat: DateFormat('yyyy-MM-dd HH:mm:ss'),
                  labelStyle: TextStyle(color: Colors.white), // Set axis label color to white
                ),
                primaryYAxis: NumericAxis(
                  labelStyle: TextStyle(color: Colors.white), // Set y-axis label color to white
                ),
                tooltipBehavior: TooltipBehavior(
                  enable: true,
                  header: '',
                  format: 'point.x\nValue: point.y',
                  canShowMarker: true,
                  textStyle: TextStyle(color: Colors.white),
                ),
                series: <CartesianSeries>[
                  LineSeries<_ChartData, DateTime>(
                    dataSource: _longitudeData,
                    xValueMapper: (_ChartData data, _) => data.timestamp,
                    yValueMapper: (_ChartData data, _) => data.value,
                    markerSettings: MarkerSettings(isVisible: true),
                    dataLabelSettings: DataLabelSettings(
                      isVisible: true,
                      textStyle: TextStyle(color: Colors.white)
                    ),
                    color: Colors.red, // Customize line color
                  ),
                ],
              ),
            ),
            // Latitude and Longitude Combined Chart
            Container(
              height: 300,
              child: SfCartesianChart(
                title: ChartTitle(
                  text: 'Latitude and Longitude Data Over Time',
                  textStyle: TextStyle(color: Colors.white),
                ),
                primaryXAxis: DateTimeAxis(
                  dateFormat: DateFormat('yyyy-MM-dd HH:mm:ss'),
                  labelStyle: TextStyle(color: Colors.white), // Set axis label color to white
                ),
                primaryYAxis: NumericAxis(
                  labelStyle: TextStyle(color: Colors.white), // Set y-axis label color to white
                ),
                tooltipBehavior: TooltipBehavior(
                  enable: true,
                  header: '',
                  format: 'point.x\nValue: point.y',
                  canShowMarker: true,
                  textStyle: TextStyle(color: Colors.white),
                ),
                series: <CartesianSeries>[
                  LineSeries<_ChartData, DateTime>(
                    dataSource: _latitudeData,
                    xValueMapper: (_ChartData data, _) => data.timestamp,
                    yValueMapper: (_ChartData data, _) => data.value,
                    markerSettings: MarkerSettings(isVisible: true),
                    dataLabelSettings: DataLabelSettings(
                      isVisible: true,
                      textStyle: TextStyle(color: Colors.white),
                    ),
                    color: Colors.green, // Customize line color for latitude
                  ),
                  LineSeries<_ChartData, DateTime>(
                    dataSource: _longitudeData,
                    xValueMapper: (_ChartData data, _) => data.timestamp,
                    yValueMapper: (_ChartData data, _) => data.value,
                    markerSettings: MarkerSettings(isVisible: true),
                    dataLabelSettings: DataLabelSettings(
                      isVisible: true,
                      textStyle: TextStyle(color: Colors.white),
                    ),
                    color: Colors.red, // Customize line color for longitude
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// Sample data class to map timestamp and value
class _ChartData {
  final DateTime timestamp;
  final double value;

  _ChartData(this.timestamp, this.value);
}