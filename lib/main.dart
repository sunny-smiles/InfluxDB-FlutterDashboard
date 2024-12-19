import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:flutter_dotenv/flutter_dotenv.dart';
import 'core/presentation/app.dart';
import 'features/measurements/presentation/providers/measurement_provider.dart';
import 'features/measurements/data/repositories/measurement_repository_impl.dart';
import 'features/measurements/data/datasources/influxdb_datasource.dart';

void main() async {
  await dotenv.load();
  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(
          create: (_) => MeasurementProvider(
            repository: MeasurementRepositoryImpl(
              dataSource: InfluxDBDataSource(),
            ),
          ),
        ),
      ],
      child: const MyApp(),
    ),
  );
}