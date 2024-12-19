import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:syncfusion_flutter_charts/charts.dart';
import 'package:intl/intl.dart';
import '../../domain/enums/measurement_type.dart';
import '../providers/measurement_provider.dart';

class MeasurementChart extends StatelessWidget {
  final String title;
  final MeasurementType measurementType;

  const MeasurementChart({
    super.key,
    required this.title,
    required this.measurementType,
  });

  Color _getChartColor() {
    switch (measurementType) {
      case MeasurementType.heading:
        return Colors.lightBlue[600]!;
      case MeasurementType.latitude:
        return Colors.green[600]!;
      case MeasurementType.longitude:
        return Colors.orange[600]!;
      case MeasurementType.ultrasonic1speed:
        return Colors.yellow[700]!;
      case MeasurementType.ultrasonic1pressure:
        return Colors.brown[500]!;
      case MeasurementType.ultrasonic2speed:
        return Colors.red[600]!;
      case MeasurementType.ultrasonic2pressure:
        return Colors.deepPurple[500]!;
      case MeasurementType.dac:
        return Colors.amber[700]!;
      case MeasurementType.ina219:
        return Colors.blueGrey[500]!;
      case MeasurementType.acs758:
        return Colors.deepOrange[600]!;
      default:
        return Colors.blue;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<MeasurementProvider>(
      builder: (context, provider, _) {
        return SfCartesianChart(
          title: ChartTitle(
            text: '$title Over Time',
            textStyle: const TextStyle(color: Colors.white),
          ),
          primaryXAxis: DateTimeAxis(
            dateFormat: DateFormat('HH:mm:ss'),
            labelStyle: const TextStyle(color: Colors.white),
          ),
          primaryYAxis: NumericAxis(
            labelStyle: const TextStyle(color: Colors.white),
            minimum: 0,
            interval: measurementType == MeasurementType.heading ? 50 : 10,
          ),
          series: <CartesianSeries>[
            LineSeries<dynamic, DateTime>(
              dataSource: provider.measurements,
              xValueMapper: (dynamic data, _) => data.timestamp,
              yValueMapper: (dynamic data, _) => data.value,
              markerSettings: const MarkerSettings(isVisible: false),
              animationDuration: 0,
              color: _getChartColor(),
            ),
          ],
        );
      },
    );
  }
}
