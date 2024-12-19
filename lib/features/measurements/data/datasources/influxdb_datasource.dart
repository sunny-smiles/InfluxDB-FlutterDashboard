import 'package:influxdb_client/api.dart';
import 'package:influxdbtesting/core/constants/env_constants.dart';
import '../../domain/enums/measurement_type.dart';

class InfluxDBDataSource {
  Future<List<FluxRecord>> fetchMeasurements(MeasurementType type) async {
    final client = InfluxDBClient(
      url: EnvConstants.url,
      token: EnvConstants.token,
      org: EnvConstants.org,
      bucket: EnvConstants.bucket,
    );

    final queryService = client.getQueryService();
    final query = _buildQuery(type);
    final records = <FluxRecord>[];

    try {
      final stream = await queryService.query(query);
      await for (final record in stream) {
        records.add(record);
      }
    } finally {
      client.close();
    }

    return records;
  }

  String _buildQuery(MeasurementType type) {
    final queryParams = _getQueryParameters(type);
    return '''
      from(bucket: "${EnvConstants.bucket}")
      |> range(start: -1m)
      |> filter(fn: (r) => r["_measurement"] == "${queryParams['measurement']}")
      |> filter(fn: (r) => r["_field"] == "${queryParams['field']}")
      |> filter(fn: (r) => r["sensor_id"] == "${queryParams['sensorId']}")
      |> filter(fn: (r) => r["sensor_type"] == "${queryParams['sensorType']}")
      |> filter(fn: (r) => r["measurement_type"] == "${queryParams['measurementType']}")     
      |> yield(name: "last")
    ''';
  }

  Map<String, String> _getQueryParameters(MeasurementType type) {
    switch (type) {
      case MeasurementType.heading:
        return {
          'measurement': 'compass_measurements',
          'field': 'value',
          'sensorId': 'compass_sensor',
          'sensorType': 'compass',
          'measurementType': 'heading',
        };
      case MeasurementType.latitude:
        return {
          'measurement': 'gps_measurements',
          'field': 'value',
          'sensorId': 'gps_sensor',
          'sensorType': 'gps',
          'measurementType': 'latitude',
        };
      case MeasurementType.longitude:
        return {
          'measurement': 'gps_measurements',
          'field': 'value',
          'sensorId': 'gps_sensor',
          'sensorType': 'gps',
          'measurementType': 'longitude',
        };
      case MeasurementType.ultrasonic1pressure:
        return {
          'measurement': 'ultrasonic_measurements',
          'field': 'value',
          'sensorId': 'ultra1',
          'sensorType': 'ultrasonic',
          'measurementType': 'pressure',
        };
      case MeasurementType.ultrasonic1speed:
        return {
          'measurement': 'ultrasonic_measurements',
          'field': 'value',
          'sensorId': 'ultra1',
          'sensorType': 'ultrasonic',
          'measurementType': 'speed',
        };
      case MeasurementType.ultrasonic2pressure:
        return {
          'measurement': 'ultrasonic_measurements',
          'field': 'value',
          'sensorId': 'ultra2',
          'sensorType': 'ultrasonic',
          'measurementType': 'pressure',
        };
      case MeasurementType.ultrasonic2speed:
        return {
          'measurement': 'ultrasonic_measurements',
          'field': 'value',
          'sensorId': 'ultra2',
          'sensorType': 'ultrasonic',
          'measurementType': 'speed',
        };
      case MeasurementType.ina219:
        return {
          'measurement': 'ina219_measurements',
          'field': 'value',
          'sensorId': 'inabattery',
          'sensorType': 'ina219',
          'measurementType': 'temperature',
        };
      case MeasurementType.acs758:
        return {
          'measurement': 'acs758_measurements',
          'field': 'value',
          'sensorId': 'acs758steer',
          'sensorType': 'acs758',
          'measurementType': 'current',
        };
      case MeasurementType.dac:
        return {
          'measurement': 'dac_measurements',
          'field': 'value',
          'sensorId': 'dac_sensor',
          'sensorType': 'dac',
          'measurementType': 'velocity',
        };
      default:
        throw UnimplementedError('Measurement type not implemented');
    }
  }
}