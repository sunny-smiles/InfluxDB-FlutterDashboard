import '../entities/measurement.dart';
import '../enums/measurement_type.dart';

abstract class MeasurementRepository {
  Future<List<Measurement>> getMeasurements(MeasurementType type);
}