import 'dart:convert';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/block_with_subblock.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class Runtime extends BlockVisitor<void> {
  factory Runtime.the() => _instance;
  Runtime._internal();

  static final Runtime _instance = Runtime._internal();
  final WebSocketChannel _channel = WebSocketChannel.connect(
    Uri.parse('ws://79.249.140.33:9090'),
  );

  void exec(List<Block> blocks) {
    for (Block block in blocks) {
      block.accept(this);
    }
  }

  @override
  void visitBlockWithSubblock(BlockWithSubblock blockWithSubblock) {
    print('Exec block with subblock');
    //blockWithSubblock.subblock.accept(this);
  }

  @override
  void visitCommentBlock(CommentBlock commentBlock) async {
    print('Exec comment Block');
    _channel.sink.add(jsonEncode({
      'op': 'subscribe',
      'topic': 'sonar_range',
      'type': 'sensor_msgs/Range'
    }));

    _channel.stream.listen(
      (dynamic data) => print(data),
      onError: (dynamic error) => print(error),
    );
  }

  @override
  void visitTestBlock(TestBlock testBlock) {
    print('Exec test block');
  }
}
