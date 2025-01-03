import 'dart:io';
import 'package:path_provider/path_provider.dart';

class FileService {
  static Future<String> saveCSV(List<int> csvData) async {
    // Get the directory to save the file
    final directory = await getApplicationDocumentsDirectory();
    String baseFileName = 'data_test';
    String extension = '.csv';
    String filePath = '${directory.path}/$baseFileName$extension';
    int fileNumber = 1;

    // Check if the file already exists, and find the next available filename
    while (await File(filePath).exists()) {
      filePath = '${directory.path}/$baseFileName($fileNumber)$extension';
      fileNumber++;
    }

    // Write the CSV data to the file
    final file = File(filePath);
    await file.writeAsBytes(csvData);

    return filePath;
  }
}
