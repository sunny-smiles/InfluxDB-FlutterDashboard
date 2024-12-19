import 'package:flutter/material.dart';
import '../../features/measurements/presentation/pages/home_page.dart';

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'InfluxDB Data Visualizer',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        scaffoldBackgroundColor: Colors.black,
        appBarTheme: AppBarTheme(
          backgroundColor: Colors.grey[900],
          titleTextStyle: const TextStyle(color: Colors.white),
          iconTheme: const IconThemeData(color: Colors.white),
        ),
      ),
      home: const HomePage(),
    );
  }
}