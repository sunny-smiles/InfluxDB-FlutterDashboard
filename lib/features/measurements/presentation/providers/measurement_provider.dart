import 'package:flutter/foundation.dart';
import 'dart:async';
import '../../domain/entities/measurement.dart';
import '../../domain/repositories/measurement_repository.dart';
import '../../domain/enums/measurement_type.dart';

class MeasurementProvider extends ChangeNotifier {
  final MeasurementRepository repository;
  List<Measurement> measurements = [];
  Timer? _timer;
  MeasurementType? currentType;

  MeasurementProvider({required this.repository});

  void startMonitoring(MeasurementType type) {
    currentType = type;
    _fetchData();
    _timer?.cancel();
    _timer = Timer.periodic(const Duration(seconds: 1), (_) => _fetchData());
  }

  Future<void> _fetchData() async {
    if (currentType == null) return;
    
    try {
      final newMeasurements = await repository.getMeasurements(currentType!);
      measurements = [...newMeasurements];
      if (measurements.length > 50) {
        measurements = measurements.sublist(measurements.length - 50);
      }
      notifyListeners();
    } catch (e) {
      print('Error fetching measurements: $e');
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }
}

