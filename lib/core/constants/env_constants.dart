import 'package:flutter_dotenv/flutter_dotenv.dart';

class EnvConstants {
  static final String url = dotenv.env['_URL'] ?? '';
  static final String token = dotenv.env['_TOKEN'] ?? '';
  static final String org = dotenv.env['_ORG'] ?? '';
  static final String bucket = dotenv.env['_BUCKET'] ?? '';
}