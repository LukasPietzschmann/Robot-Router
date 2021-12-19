import 'package:flutter/material.dart';
import 'package:robot_router/custom_blocks/block_with_subblock.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';

enum BlockType { t_action, t_passive, t_event, t_control }

Future<Block?> selectBlockFromList(BuildContext context) {
  return showModalBottomSheet<Block>(
    context: context,
    isDismissible: true,
    enableDrag: false,
    shape: ContinuousRectangleBorder(
      borderRadius: BorderRadius.circular(20),
    ),
    builder: (BuildContext context) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Expanded(
                child: ListView(
                    padding: const EdgeInsets.all(10),
                    children: Block.allBlocks().map((List<Block> blocks) {
                      return Wrap(direction: Axis.vertical, children: <Widget>[
                        Text(blocks[0].typeName),
                        ...blocks.map((Block block) => TextButton(
                            onPressed: () => Navigator.pop(context, block),
                            child: Text('    ' + block.name)))
                      ]);
                    }).toList()))
          ],
        ),
      );
    },
  );
}

abstract class BlockVisitor<T> {
  T visitTestBlock(TestBlock testBlock);
  T visitCommentBlock(CommentBlock commentBlock);
  T visitBlockWithSubblock(BlockWithSubblock blockWithSubblock);
}

abstract class Block extends StatefulWidget {
  const Block({Key? key}) : super(key: key);

  String get name;
  BlockType get type;
  String get typeName {
    return <String>[
      'Action Block',
      'Passive Block',
      'Event Block',
      'Control Block'
    ][type.index];
  }

  T accept<T>(BlockVisitor<T> visitor);

  static List<List<Block>> allBlocks() {
    final List<Block> blocks = <Block>[
      const TestBlock(),
      const CommentBlock(),
      const BlockWithSubblock(),
    ];
    List<List<Block>> result = List<List<Block>>.generate(
        BlockType.values.length, (int index) => <Block>[]);
    for (Block block in blocks) {
      result[block.type.index].add(block);
    }
    result.removeWhere((List<Block> blocks) => blocks.isEmpty);
    return result;
  }

  @override
  BlockState createState();
}

abstract class BlockState extends State<Block> {
  bool _isCurrentlyRunning = false;

  bool get isCurrentlyRunning {
    return _isCurrentlyRunning;
  }

  set isCurrentlyRunning(bool newVal) {
    setState(() {
      _isCurrentlyRunning = newVal;
    });
  }

  Widget render();

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.all(10),
      shape: ContinuousRectangleBorder(
        side: BorderSide(
            color: _isCurrentlyRunning ? Colors.green : Colors.black, width: 1),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Padding(padding: const EdgeInsets.all(15), child: render()),
    );
  }
}
