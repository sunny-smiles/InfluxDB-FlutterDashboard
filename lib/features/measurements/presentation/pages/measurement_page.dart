import 'package:flutter/material.dart' hide NavigationDrawer;
import 'package:provider/provider.dart';
//import 'package:syncfusion_flutter_charts/charts.dart';
//import 'package:intl/intl.dart';
import '../../domain/enums/measurement_type.dart';
import '../providers/measurement_provider.dart';
import '../widgets/navigation_drawer.dart';
import '../widgets/measurement_chart.dart';
//import 'dart:io';
//import 'package:influxdbtesting/features/measurements/data/datasources/influxdb_datasource.dart';

class MeasurementPage extends StatefulWidget {
  final MeasurementType measurementType;
  final String title;

  const MeasurementPage({
    super.key,
    required this.measurementType,
    required this.title,
  });

  @override
  State<MeasurementPage> createState() => _MeasurementPageState();
}

class _MeasurementPageState extends State<MeasurementPage> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<MeasurementProvider>().startMonitoring(widget.measurementType);


    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      drawer: const NavigationDrawer(),
      body: Container(
        padding: const EdgeInsets.all(16),
        child: MeasurementChart(
          title: widget.title,
          measurementType: widget.measurementType,
        ),
      ),
    );
  }
}
