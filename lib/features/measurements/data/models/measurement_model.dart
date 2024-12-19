import 'package:influxdb_client/api.dart';
import '../../domain/entities/measurement.dart';

class MeasurementModel extends Measurement {
  MeasurementModel({
    required DateTime timestamp,
    required double value,
  }) : super(timestamp: timestamp, value: value);

  factory MeasurementModel.fromFluxRecord(FluxRecord record) {
    return MeasurementModel(
      timestamp: DateTime.parse(record['_time']).toLocal(),
      value: record['_value'].toDouble(),
    );
  }
}