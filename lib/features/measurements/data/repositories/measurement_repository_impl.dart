import '../../domain/entities/measurement.dart';
import '../../domain/repositories/measurement_repository.dart';
import '../../domain/enums/measurement_type.dart';
import '../datasources/influxdb_datasource.dart';
import '../models/measurement_model.dart';

class MeasurementRepositoryImpl implements MeasurementRepository {
  final InfluxDBDataSource dataSource;

  MeasurementRepositoryImpl({required this.dataSource});

  @override
  Future<List<Measurement>> getMeasurements(MeasurementType type) async {
    final records = await dataSource.fetchMeasurements(type);
    return records.map((record) => MeasurementModel.fromFluxRecord(record)).toList();
  }
}