import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
enum BlockType { t_action, t_passive, t_event, t_control }
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

  static List<List<Block>> allBlocks() {
    final List<Block> blocks = <Block>[
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
