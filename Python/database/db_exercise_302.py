"""
데이터베이스 연습 #302
SQLite CRUD 실습
"""
import sqlite3

def setup_302():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_302 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_302_0', 942.07, 'B'), ('item_302_1', 773.46, 'C'), ('item_302_2', 215.21, 'C'), ('item_302_3', 494.11, 'B'), ('item_302_4', 355.32, 'C'), ('item_302_5', 280.72, 'A'), ('item_302_6', 577.74, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_302 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_302()
    cursor = conn.cursor()

    print(f"DB 연습 #302")
    cursor.execute('SELECT * FROM items_302')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_302 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
